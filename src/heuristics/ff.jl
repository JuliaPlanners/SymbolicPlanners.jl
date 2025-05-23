## FastForward (FF) delete-relaxation heuristic ##
export FFHeuristic

"""
    FFHeuristic()

A relaxed planning graph heuristic introduced by the FastForward planner [1]. 
Similar to [`HSPHeuristic`](@ref), this heuristic precomputes a graph that
stores the dependencies between all ground actions and plan-relevant conditions.

To estimate the distance to the goal, a shortest-path search is performed in the
graph, starting from the conditions that are true in the current state, and 
ending when all the goal conditions are reached. Once a condition is achieved
by an action, it is considered to remain true through the rest of the search,
hence the relaxed nature of the heuristic. A plan that achieves the goal
conditions is reconstructed by following the action back-pointers for each
achieved condition, and the cost of this plan is used as the heuristic estimate.

This implementation supports domains with negative preconditions, disjunctive
preconditions (i.e., `or`, `exists`), and functional preconditions (e.g.
numeric comparisons, or other Boolean-valued functions of non-Boolean fluents).
Functional preconditions are handled by (optimistically) assuming they become
true once a constituent fluent is modified by some action.

[1] J. Hoffmann, "FF: The Fast-Forward Planning System," AI Magazine, vol. 22,
no. 3, pp. 57–57, Sep. 2001, <https://doi.org/10.1609/aimag.v22i3.1572>.
"""
mutable struct FFHeuristic <: Heuristic
    dynamic_goal::Bool # Flag whether goal-relevant information is dynamic
    spec_obj_id::Union{Nothing,UInt} # Object ID of most recently precomputed spec
    goal_obj_id::Union{Nothing,UInt} # Object ID of most recently precomputed goal
    goal_hash::Union{Nothing,UInt} # Hash of most recently computed goal terms
    statics::Vector{Symbol} # Static domain fluents 
    graph::PlanningGraph # Precomputed planning graph
    search_state::PlanningGraphSearchState # Preallocated search state
    FFHeuristic() = new()
end

function Base.show(io::IO, h::FFHeuristic)
    print(io, summary(h), "(precomputed=$(is_precomputed(h)))")
end

is_precomputed(h::FFHeuristic) = isdefined(h, :graph)

function precompute!(h::FFHeuristic,
                     domain::Domain, state::State)
    # If goal specification is not provided, assume dynamic goal
    h.dynamic_goal = true
    h.spec_obj_id = nothing
    h.goal_obj_id = nothing
    h.goal_hash = nothing
    h.statics = infer_static_fluents(domain)
    h.graph = build_planning_graph(domain, state; statics=h.statics)
    h.search_state = PlanningGraphSearchState(h.graph)
    return h
end

function precompute!(h::FFHeuristic,
                     domain::Domain, state::State, spec::Specification)
    # If goal specification is provided, assume non-dynamic goal
    goals = get_goal_terms(spec)
    h.dynamic_goal = false
    h.spec_obj_id = objectid(spec)
    h.goal_obj_id = objectid(goals)
    h.goal_hash = hash(goals)
    h.statics = infer_static_fluents(domain)
    h.graph = build_planning_graph(domain, state, spec; statics=h.statics)
    h.search_state = PlanningGraphSearchState(h.graph)
    return h
end

function update_spec!(h::FFHeuristic,
                      domain::Domain, state::State, spec::Specification)
    !h.dynamic_goal && return h
    objectid(spec) == h.spec_obj_id && return h
    h.spec_obj_id = objectid(spec)
    goals = get_goal_terms(spec)
    objectid(goals) == h.goal_obj_id && return h
    h.goal_obj_id = objectid(goals)
    goal_hash = hash(goals)
    goal_hash == h.goal_hash && return h
    h.goal_hash = goal_hash
    h.graph = update_pgraph_goal!(h.graph, domain, state, spec;
                                  statics=h.statics)
    return h
end

function compute(h::FFHeuristic,
                 domain::Domain, state::State, spec::Specification)
    # If necessary, update planning graph with new goal
    update_spec!(h, domain, state, spec)
    # Compute achievers to each condition node of the relaxed planning graph
    init_pgraph_search!(h.search_state, h.graph, domain, state)
    search_state, goal_idx, _ =
        run_pgraph_search!(h.search_state, h.graph, spec; compute_achievers=true)
    costs = search_state.cond_costs
    achievers = search_state.cond_achievers
    # Return infinity if goal is not reached
    if isnothing(goal_idx) return Inf32 end
    # Initialize queue
    queue = Int[]
    goal_parents = h.graph.act_parents[goal_idx]
    ff_add_parent_conds_to_queue!(queue, goal_parents, costs)
    # Extract cost of relaxed plan via backward chaining
    plan_cost = 0.0f0
    act_idxs = Int[]
    while !isempty(queue)
        cond_idx = popfirst!(queue)
        act_idx = achievers[cond_idx]
        act_idx == -1 && continue # Skip conditions achieved from the start
        act_idx in act_idxs && continue # Skip actions that already appeared
        push!(act_idxs, act_idx)
        if act_idx > h.graph.n_axioms # Add cost to plan for non-axioms
            plan_cost += has_action_cost(spec) ?
                get_action_cost(spec, h.graph.actions[act_idx].term) : 1
        end
        # Push supporting condition indices onto queue
        act_parents = h.graph.act_parents[act_idx]
        ff_add_parent_conds_to_queue!(queue, act_parents, costs)
    end
    # TODO: Store helpful actions
    # Return cost of relaxed plan as heuristic estimate
    return plan_cost
end

"Adds parent conditions of an action to the queue."
function ff_add_parent_conds_to_queue!(queue, act_parents, costs)
    for precond_parents in act_parents
        if length(precond_parents) == 1
            push!(queue, precond_parents[1])
        else
            _, min_idx = findmin(costs[precond_parents])
            min_cond_idx = precond_parents[min_idx]
            push!(queue, min_cond_idx)
        end
    end
    return queue
end
