## The LM-Cut Heuristic ##
export LMCut

"""
    LMCut()

A landmark-based heuristic, which builds on top of the relaxed planning graph 
heuristic [`HMax`](@ref). This heuristic iteratively finds sets of actions
through which any relaxed plan must pass (action landmarks), adding the cost of 
the least costly landmark to the total heuristic value. This cost is then
subtracted from the cost of each landmark, and the process is repeated until the
cost of the relaxed plan is driven to zero. The value of the heuristic is thus
the sum of the minimum cost actions across all sets of landmarks.
"""
mutable struct LMCut <: Heuristic
    dynamic_goal::Bool # Flag whether goal-relevant information is dynamic
    goal_hash::Union{Nothing,UInt} # Hash of most recently pre-computed goal
    statics::Vector{Symbol} # Static domain fluents
    graph::PlanningGraph # Precomputed planning graph
    action_costs::Vector{Float32} # Precomputed action costs
    LMCut() = new()
end

is_precomputed(h::LMCut) = isdefined(h, :graph)

function Base.show(io::IO, h::LMCut)
    is_precomputed_str = "precomputed=$(is_precomputed(h))"
    print(io, summary(h), "(",  "", is_precomputed_str, ")")
end

function precompute!(h::LMCut, domain::Domain, state::State)
    # If goal specification is not provided, assume dynamic goal
    h.dynamic_goal = true
    h.goal_hash = nothing
    # Precompute static domain fluents and planning graph
    h.statics = infer_static_fluents(domain)
    h.graph = build_planning_graph(domain, state; statics=h.statics)
    # Precompute cost of each action
    n_actions = length(h.graph.actions)
    h.action_costs = map(eachindex(h.graph.actions)) do act_idx
        if h.graph.n_axioms < act_idx <= n_actions - h.graph.n_goals
            return 1.0f0
        else
            return 0.0f0
        end
    end
    return h
end

function precompute!(h::LMCut, domain::Domain, state::State, spec::Specification)
    # If goal specification is provided, assume non-dynamic goal
    h.dynamic_goal = false
    h.goal_hash = hash(get_goal_terms(spec))
    # Precompute static domain fluents and planning graph
    h.statics = infer_static_fluents(domain)
    h.graph = build_planning_graph(domain, state, spec; statics=h.statics)
    # Precompute cost of each action
    n_actions = length(h.graph.actions)
    h.action_costs = map(enumerate(h.graph.actions)) do (act_idx, act)
        if h.graph.n_axioms < act_idx <= n_actions - h.graph.n_goals
            return has_action_cost(spec) ?
                Float32(get_action_cost(spec, act.term)) : 1.0f0
        else
            return 0.0f0
        end
    end
    return h
end

function compute(h::LMCut, domain::Domain, state::State, spec::Specification)
    # If necessary, update planning graph with new goal
    if h.dynamic_goal && hash(get_goal_terms(spec)) != h.goal_hash
        h.graph = update_pgraph_goal!(h.graph, domain, state, spec;
                                      statics=h.statics)
        h.goal_hash = hash(get_goal_terms(spec))
        n_actions = length(h.graph.actions)
        resize!(h.action_costs, n_actions)
        for (act_idx, act) in enumerate(h.graph.actions)
            if h.graph.n_axioms < act_idx <= n_actions - h.graph.n_goals
                h.action_costs[act_idx] = has_action_cost(spec) ?
                    Float32(get_action_cost(spec, act.term)) : 1.0f0
            else
                h.action_costs[act_idx] = 0.0f0
            end
        end
    end
    # Compute set of initial facts
    init_idxs = pgraph_init_idxs(h.graph, domain, state)
    # Calculate relaxed costs of facts and the h-max value
    cond_costs, _, goal_idx, goal_cost =
        relaxed_pgraph_search(domain, state, spec, maximum, h.graph;
                              action_costs = h.action_costs)
    # Terminate early if goal is unreachable
    goal_cost == Inf32 && return goal_cost
    # Iteratively find landmark cuts and sum their costs
    hval = 0.0f0
    action_costs = copy(h.action_costs)
    for _ in 1:length(h.graph.actions)
        # Find the supporters for each action
        supporters = find_supporters(h.graph, cond_costs)
        # Construct the justification graph
        jgraph = build_justification_graph(h.graph, supporters, action_costs)
        # Extract the goal zone
        goal_zone = extract_goal_zone(jgraph)
        # Extract the pregoal zone, landmarks, and their cost
        pregoal_zone, landmark_idxs, landmark_cost =
            extract_pregoal_zone_and_landmarks(jgraph, goal_zone,
                                               init_idxs, action_costs)
        # Update heuristic value and action costs
        hval += landmark_cost
        for idx in landmark_idxs
            action_costs[idx] -= landmark_cost
        end
        # Re-calculate relaxed costs to each fact
        cond_costs, _, goal_idx, goal_cost =
            relaxed_pgraph_search(domain, state, spec, maximum, h.graph;
                                  action_costs = action_costs)
        # Terminate once goal cost has been reduced to zero
        iszero(goal_cost) && break
    end
    return hval
end

"Finds the most costly precondition (i.e. supporter) of each action."
function find_supporters(pgraph::PlanningGraph, cond_costs::Vector{T}) where {T <: Real}
    supporters = map(eachindex(pgraph.actions)) do act_idx
        max_cond_idx = nothing
        max_precond_val = typemin(T)
        # Find most costly precondition clause
        for precond_idxs in pgraph.act_parents[act_idx]
            min_cond_idx = nothing
            min_cond_val = typemax(T)
            # Find least costly condition in the disjunctive clause
            for cond_idx in precond_idxs
                if isnothing(min_cond_idx) || cond_costs[cond_idx] < min_cond_val
                    min_cond_idx = cond_idx
                    min_cond_val = cond_costs[cond_idx]
                end
            end
            if isnothing(max_cond_idx) || min_cond_val > max_precond_val
                max_cond_idx = min_cond_idx
                max_precond_val = min_cond_val
            end
        end
        return max_cond_idx::Int
    end
    return supporters
end

"Justification graph used by the LMCut heuristic."
struct JustificationGraph
    fadjlist::Vector{Vector{Tuple{Int, Int}}}
    badjlist::Vector{Vector{Int}}
end

function JustificationGraph(n_conditions::Int)
    fadjlist = [Tuple{Int, Int}[] for _ in 1:n_conditions+1]
    badjlist = [Int[] for _ in 1:n_conditions+1]
    return JustificationGraph(fadjlist, badjlist)
end

"Constructs a justification graph from the relaxed planning graph."
function build_justification_graph(
    pgraph::PlanningGraph, supporters::Vector{Int}, action_costs::Vector{<:Real}
)
    n_conditions = length(pgraph.conditions)
    n_actions = length(pgraph.actions)
    last_nongoal_idx = n_actions - pgraph.n_goals
    # Construct a new justification graph
    jgraph = JustificationGraph(n_conditions)
    # Add edges from supporter of each action to child conditions of each action
    for act_idx in 1:n_actions
        parent_idx = supporters[act_idx]
        act_cost = action_costs[act_idx]
        for child_idx in pgraph.act_children[act_idx]
            push!(jgraph.fadjlist[parent_idx], (act_idx, child_idx))
            act_cost == 0 && push!(jgraph.badjlist[child_idx], parent_idx)
        end
        # Add edges to from goal conditions to dummy goal node
        if act_idx > last_nongoal_idx
            push!(jgraph.fadjlist[parent_idx], (act_idx, n_conditions + 1))
            push!(jgraph.badjlist[n_conditions + 1], parent_idx)
        end
    end
    return jgraph
end

"Extract goal zone from the justification graph."
function extract_goal_zone(jgraph::JustificationGraph)
    goal_zone = Set{Int}()
    goal_idx = length(jgraph.fadjlist)
    queue = Int[goal_idx]
    while !isempty(queue)
        cond_idx = popfirst!(queue)
        for parent_idx in jgraph.badjlist[cond_idx]
            if !in(parent_idx, goal_zone)
                push!(goal_zone, parent_idx)
                push!(queue, parent_idx)
            end
        end
    end
    return goal_zone
end

"Extract pregoal zone and action landmarks from the justification graph."
function extract_pregoal_zone_and_landmarks(
    jgraph::JustificationGraph, goal_zone::Set{Int},
    init_idxs::BitVector, action_costs::Vector{<:Real}
)
    landmark_idxs = Set{Int}()
    landmark_cost = Inf32
    queue = findall(init_idxs)
    pregoal_zone = Set{Int}(queue)
    while !isempty(queue)
        cond_idx = popfirst!(queue)
        for (act_idx, child_idx) in jgraph.fadjlist[cond_idx]
            if child_idx in goal_zone
                # Add action to set of landmarks
                push!(landmark_idxs, act_idx)
                act_cost = action_costs[act_idx]
                landmark_cost = min(landmark_cost, act_cost)                    
            elseif !(child_idx in pregoal_zone)
                # Add node to pregoal zone and queue
                push!(pregoal_zone, child_idx)
                push!(queue, child_idx)
            end
        end
    end
    return pregoal_zone, landmark_idxs, landmark_cost
end
