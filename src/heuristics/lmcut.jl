## The LM-Cut Heuristic ##
export LMCut

"""
    LMCut()

A landmark-based heuristic [1], which builds on top of the relaxed planning
graph heuristic [`HMax`](@ref). This heuristic iteratively finds sets of
actions through which any relaxed plan must pass (action landmarks), adding the
cost of  the least costly landmark to the total heuristic value. This cost is
then subtracted from the cost of each landmark, and the process is repeated
until the cost of the relaxed plan is driven to zero. The value of the heuristic
is thus the sum of the minimum cost actions across all sets of landmarks.

[1] B. Bonet and H. Geffner, "Landmarks, Critical Paths and Abstractions: What's
the Difference Anyway?,"  ICAPS (2009), vol. 19 no. 1, pp. 162-169.
<https://doi.org/10.1609/icaps.v19i1.13370>
"""
mutable struct LMCut <: Heuristic
    dynamic_goal::Bool # Flag whether goal-relevant information is dynamic
    goal_hash::Union{Nothing,UInt} # Hash of most recently pre-computed goal
    statics::Vector{Symbol} # Static domain fluents
    graph::PlanningGraph # Precomputed planning graph
    search_state::PlanningGraphSearchState # Preallocated search state
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
    h.search_state = PlanningGraphSearchState(h.graph)
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
    h.search_state = PlanningGraphSearchState(h.graph)
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
    # Calculate relaxed costs of facts and the h-max value
    init_pgraph_search!(h.search_state, h.graph, domain, state)
    fill!(h.search_state.act_pathcosts, -Inf32)
    search_state, goal_idx, goal_cost =
        run_pgraph_search!(h.search_state, h.graph, spec;
                           compute_supporters = true,
                           action_costs = h.action_costs)
    # Terminate early if goal is unreachable
    goal_cost == Inf32 && return goal_cost
    # Iteratively find landmark cuts and sum their costs
    hval = 0.0f0
    init_conds = search_state.init_conds
    action_costs = copy(h.action_costs)
    for _ in 1:length(h.graph.actions)
        # Construct the justification graph
        supporters = search_state.act_supporters
        jgraph = build_justification_graph(h.graph, supporters, action_costs)
        # Extract the goal zone
        goal_zone = extract_goal_zone(jgraph)
        # Extract the pregoal zone, landmarks, and their cost
        pregoal_zone, landmark_idxs, landmark_cost =
            extract_pregoal_zone_and_landmarks(jgraph, goal_zone,
                                               init_conds, action_costs)
        # Update heuristic value, action costs and search queue
        hval += landmark_cost
        for act_idx in landmark_idxs
            action_costs[act_idx] -= landmark_cost
        end
        # Update fact costs and actions supporters
        search_state, goal_idx, goal_cost =
            update_pgraph_costs!(h.search_state, h.graph,
                                 landmark_idxs, action_costs)
        # Terminate once goal cost has been reduced to zero
        iszero(goal_cost) && break
    end
    return hval
end

"Update costs of facts in the planning graph given updated landmark costs."
function update_pgraph_costs!(
    search_state::PlanningGraphSearchState, graph::PlanningGraph, 
    landmark_idxs::Set{Int}, action_costs::Vector{<:Real}
)
    # Unpack search state
    cond_costs = search_state.cond_costs
    act_pathcosts = search_state.act_pathcosts
    act_supporters = search_state.act_supporters
    queue = search_state.queue

    # Reinitialize search queue with cost-reduced conditions
    empty!(queue)
    for act_idx in landmark_idxs
        path_cost = act_pathcosts[act_idx]
        next_cost = path_cost + action_costs[act_idx]
        for c_idx in graph.act_children[act_idx]
            # Update cost and place on queue if cost decreased
            next_cost < cond_costs[c_idx] || continue
            cond_costs[c_idx] = next_cost
            enqueue!(queue, c_idx, next_cost)
        end
    end

    # Propagate reduced costs to landmark descendants
    goal_idx, goal_cost = nothing, Inf32
    last_nongoal_idx = length(graph.actions) - graph.n_goals
    while !isempty(queue)
        # Dequeue nearest fact/condition
        cond_idx, cond_cost = dequeue_pair!(queue)
        # Skip if cost is greater than stored value
        cond_cost > cond_costs[cond_idx] && continue
        # Iterate over child actions
        for (act_idx, _) in graph.cond_children[cond_idx]
            # Skip if condition is not the supporter of this action
            supporter_idx = act_supporters[act_idx]
            supporter_idx == cond_idx || continue
            # Skip if supporter cost is not lower than it was previously
            supporter_cost = cond_cost
            prev_supporter_cost = act_pathcosts[act_idx]
            supporter_cost < prev_supporter_cost || continue
            # Update supporter and supporter cost for action
            for precond_idxs in graph.act_parents[act_idx]
                min_precond_idx = first(precond_idxs)
                min_precond_cost = cond_costs[min_precond_idx]
                if length(precond_idxs) > 1
                    # Find least costly condition in disjunctive precondition
                    for p_idx in Iterators.rest(precond_idxs, 2)
                        cond_costs[p_idx] < min_precond_cost || continue
                        min_precond_idx = p_idx
                        min_precond_cost = cond_costs[p_idx]
                    end
                end
                # Set as new supporter if higher in cost than old supporter
                min_precond_cost > supporter_cost || continue
                act_supporters[act_idx] = min_precond_idx
                supporter_cost = min_precond_cost
            end
            if act_idx > last_nongoal_idx
                # Terminate if goal is reached
                act_pathcosts[act_idx] = supporter_cost
                goal_idx, goal_cost = act_idx, supporter_cost
                break
            elseif supporter_cost < prev_supporter_cost
                # Update action path cost, and place child conditions on queue
                act_pathcosts[act_idx] = supporter_cost
                next_cost = supporter_cost + action_costs[act_idx]
                for c_idx in graph.act_children[act_idx]
                    next_cost < cond_costs[c_idx] || continue
                    cond_costs[c_idx] = next_cost
                    enqueue!(queue, c_idx, next_cost)
                end
            end
        end
    end

    return search_state, goal_idx, goal_cost
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
        parent_idx == -1 && continue
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
