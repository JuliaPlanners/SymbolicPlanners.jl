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
    act_costs::Vector{Float32} # Precomputed action costs
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
    h.act_costs = map(eachindex(h.graph.actions)) do act_idx
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
    h.act_costs = map(enumerate(h.graph.actions)) do (act_idx, act)
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
        resize!(h.act_costs, n_actions)
        for (act_idx, act) in enumerate(h.graph.actions)
            if h.graph.n_axioms < act_idx <= n_actions - h.graph.n_goals
                h.act_costs[act_idx] = has_action_cost(spec) ?
                    Float32(get_action_cost(spec, act.term)) : 1.0f0
            else
                h.act_costs[act_idx] = 0.0f0
            end
        end
    end
    # Calculate relaxed costs of facts and action supporters
    init_pgraph_search!(h.search_state, h.graph, domain, state)
    fill!(h.search_state.act_pathcosts, -Inf32)
    _, goal_idx, goal_cost =
        run_pgraph_search!(h.search_state, h.graph, spec;
                           compute_supporters = true, act_costs = h.act_costs)
    # Terminate early if goal is unreachable
    goal_cost == Inf32 && return goal_cost
    # Iteratively find landmark cuts and sum their costs
    hval = 0.0f0
    act_costs = copy(h.act_costs)
    landmarks = Int[]
    in_goal_zone = falses(length(h.graph.conditions))
    in_pregoal_zone = falses(length(h.graph.conditions))
    for _ in 1:length(h.graph.actions)
        # Extract action landmarks and their cost
        landmarks, landmark_cost =
            extract_landmark_cut(h.search_state, h.graph, goal_idx, act_costs,
                                 landmarks, in_goal_zone, in_pregoal_zone)
        # Update heuristic value and action costs
        hval += landmark_cost
        for act_idx in landmarks
            act_costs[act_idx] -= landmark_cost
        end
        # Update fact costs and action supporters
        _, goal_idx, goal_cost =
            update_pgraph_costs!(h.search_state, h.graph, landmarks, act_costs)
        # Terminate once goal cost has been reduced to zero
        iszero(goal_cost) && break
        # Empty buffers for next iteration
        empty!(landmarks)
        fill!(in_goal_zone, false)
    end
    return hval
end

"Extract landmark cut from the implicit justification graph."
function extract_landmark_cut(
    search_state::PlanningGraphSearchState, graph::PlanningGraph,
    goal_idx::Int, act_costs::Vector{<:Real},
    landmark_idxs::Vector{Int} = Int[],
    in_goal_zone::BitVector = falses(length(graph.conditions)),
    in_pregoal_zone::BitVector = falses(length(graph.conditions))
)
    # Mark goal zone (nodes in justification graph with zero cost to the goal)
    goal_supporter_idx = search_state.act_supporters[goal_idx]
    in_goal_zone[goal_supporter_idx] = true
    queue = Int[goal_supporter_idx]
    while !isempty(queue)
        cond_idx = popfirst!(queue)
        # Iterate over supporters of parent actions with zero cost
        for act_idx in graph.cond_parents[cond_idx]
            iszero(act_costs[act_idx]) || continue
            parent_idx = search_state.act_supporters[act_idx]
            if parent_idx != -1 && !in_goal_zone[parent_idx]
                in_goal_zone[parent_idx] = true
                push!(queue, parent_idx)
            end
        end
    end
    # Find landmark cut (actions between the pregoal zone and the goal zone)
    landmark_cost = Inf32
    copyto!(in_pregoal_zone, search_state.init_conds)
    queue = findall(search_state.init_conds)
    while !isempty(queue)
        cond_idx = popfirst!(queue)
        # Iterate over supported child actions
        for (act_idx, _) in graph.cond_children[cond_idx]
            search_state.act_supporters[act_idx] == cond_idx || continue
            # Iterate over child conditions of supported action
            for child_idx in graph.act_children[act_idx]
                if in_goal_zone[child_idx]
                    # Add action to set of landmarks if it crosses the zones
                    push!(landmark_idxs, act_idx)
                    act_cost = act_costs[act_idx]
                    landmark_cost = min(landmark_cost, act_cost)
                elseif !in_pregoal_zone[child_idx]
                    # Add node to pregoal zone and search queue
                    in_pregoal_zone[child_idx] = true
                    push!(queue, child_idx)
                end
            end
        end
    end
    # Remove duplicate landmarks
    unique!(landmark_idxs)
    return landmark_idxs, landmark_cost
end

"Update costs of facts in the planning graph given updated landmark costs."
function update_pgraph_costs!(
    search_state::PlanningGraphSearchState, graph::PlanningGraph, 
    landmark_idxs::Vector{Int}, act_costs::Vector{<:Real}
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
        next_cost = path_cost + act_costs[act_idx]
        for c_idx in graph.act_children[act_idx]
            # Update cost and place on queue if cost decreased
            next_cost < cond_costs[c_idx] || continue
            cond_costs[c_idx] = next_cost
            enqueue!(queue, c_idx, next_cost)
        end
    end
    # Propagate reduced costs to landmark descendants
    goal_idx, goal_cost = nothing, Inf32
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
            # Update action path cost, and place child conditions on queue
            supporter_cost < prev_supporter_cost || continue
            act_pathcosts[act_idx] = supporter_cost
            next_cost = supporter_cost + act_costs[act_idx]
            for c_idx in graph.act_children[act_idx]
                next_cost < cond_costs[c_idx] || continue
                cond_costs[c_idx] = next_cost
                enqueue!(queue, c_idx, next_cost)
            end
        end
    end
    # Find least costly goal action
    goal_idx, goal_cost = nothing, Inf32
    n_actions = length(graph.actions)
    last_nongoal_idx = n_actions - graph.n_goals
    for act_idx in last_nongoal_idx+1:n_actions
        path_cost = act_pathcosts[act_idx]
        if path_cost !== -Inf32 && path_cost < goal_cost
            goal_idx, goal_cost = act_idx, path_cost
        end
    end
    return search_state, goal_idx, goal_cost
end
