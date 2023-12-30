export RealTimeHeuristicSearch, RTHS

include("reusable_tree.jl")

"""
    RealTimeHeuristicSearch(;
        heuristic::Heuristic = GoalCountHeuristic(),
        n_iters::Int = 10,
        max_nodes::Int = 50,
        reuse_paths::Bool = true
        kwargs...
    )

    RealTimeHeuristicSearch(
        planner::ForwardPlanner,
        n_iters::Int = 10,
        reuse_paths::Bool = true
    )

A real time heuristic search algorithm (`RTHS` for short) [1]. Similar to
`RTDP`, except that instead of greedy rollouts, lookahead heuristic search is
performed from each neighbor of the current state (up to a budget of
`max_nodes`). States encountered during search are used to update the neighbors'
value estimates, then a simulated step is taken from the current state to the
highest-value neighbor. This is repeated for `n_iters`,  with future searches
using the updated value estimates as more informed heuristics. Any `kwargs`
are passed to the [`ForwardPlanner`](@ref) used internally for search.

Each time heuristic search is performed, state values are updated by
backpropagating the value estimate of the terminal search node ``t``
(i.e., the node about to be expanded when forward search terminates due to
reaching the goal or a resource limit) to all closed nodes ``s`` in
the search tree:

```math
V(s) = -[f(t) - g(s)] = -[g(t) + h(t) - g(s)] = -[(g(t) - g(s)) + h(t)]
```

where ``g(t)`` is the path cost from the root of the search tree to ``t``,
``h(t)`` is the heuristic goal-distance estimate for ``t``, and
``f(t) = g(t) + h(t)``, the priority value for the terminal node.

The updated value of ``V(s)`` represents the (negative) estimated cost from
``s`` to the goal, computed by summing the distance from ``s`` to ``t`` with the
estimated distance of ``t`` to the goal. This update rule follows a variant of
Learning Real Time A* (LRTA) [1] called Real-Time Adaptive A* (RTAA) [2]. Note
that the RTAA* update rule requires initial heuristic values to be consistent in
order for the updated heuristic values to remain admissible (and consistent).

If `reuse_paths` is set to `true` (the default), then every time a path to the
goal is found, it is stored in a reusable tree data structure. Following
Tree Adaptive A* (TAA) [3], subsequent searches will terminate once they
encounter a path in the tree, reducing the number of node expansions.
This optimization also requires a consistent initial heuristic to preserve
optimality.

Both the value estimates and the reusable tree are returned
as a [`ReusableTreePolicy`](@ref).  Otherwise, if `reuse_paths` is set to
`false`, a [`TabularVPolicy`](@ref) is returned, which contains
only the value estimates. While this planner returns a policy, it expects
a deterministic domain as input.

[1] R. E. Korf, "Real-Time Heuristic Search," Artificial Intelligence, vol. 42,
no. 2, pp. 189–211, Mar. 1990, <https://doi.org/10.1016/0004-3702(90)90054-4>.

[2] S. Koenig and M. Likhachev, "Real-Time Adaptive A*," AAMAS (2006),
pp. 281–288. <https://doi.org/10.1145/1160633.1160682>.

[3] C. Hernández, X. Sun, S. Koenig, and P. Meseguer, "Tree Adaptive A*," 
AAMAS (2011), pp. 123–130. <https://dl.acm.org/doi/abs/10.5555/2030470.2030488>.
"""
@auto_hash_equals mutable struct RealTimeHeuristicSearch{
    P <: ForwardPlanner
} <: Planner
    planner::P
    n_iters::Int
    reuse_paths::Bool
    callback::Union{Nothing, Function}
end

const RTHS = RealTimeHeuristicSearch
@doc (@doc RealTimeHeuristicSearch) RTHS

function RealTimeHeuristicSearch(
    planner::ForwardPlanner;
    n_iters::Int = 10,
    reuse_paths::Bool = true,
    verbose::Bool = false,
    callback = verbose ? LoggerCallback() : nothing,
)
    planner = copy(planner)
    planner.save_search = true
    return RealTimeHeuristicSearch(planner, n_iters, reuse_paths, callback)
end

function RealTimeHeuristicSearch(;
    n_iters::Int = 10,
    max_nodes::Int = 50,
    reuse_paths::Bool = true,
    verbose::Bool = false,
    callback = verbose ? LoggerCallback() : nothing,
    kwargs...
)
    planner = ForwardPlanner(;max_nodes=max_nodes, save_search=true, kwargs...)
    return RealTimeHeuristicSearch(planner, n_iters, reuse_paths, callback)
end

function RealTimeHeuristicSearch(
    heuristic::Heuristic;
    n_iters::Int = 10,
    max_nodes::Int = 50,
    reuse_paths::Bool = true,
    verbose::Bool = false,
    callback = verbose ? LoggerCallback() : nothing,
    kwargs...
)
    planner = ForwardPlanner(;heuristic=heuristic, max_nodes=max_nodes,
                              save_search=true, kwargs...)
    return RealTimeHeuristicSearch(planner, n_iters, reuse_paths, callback)
end

function Base.getproperty(planner::RealTimeHeuristicSearch, name::Symbol)
    return name in (:planner, :n_iters, :reuse_paths, :callback) ?
        getfield(planner, name) : getproperty(planner.planner, name)
end

function Base.setproperty!(planner::RealTimeHeuristicSearch, name::Symbol, val)
    return name in (:planner, :n_iters, :reuse_paths, :callback) ?
        setfield!(planner, name, val) : setproperty!(planner.planner, name, val)
end

function Base.copy(p::RealTimeHeuristicSearch)
    return RealTimeHeuristicSearch(copy(p.planner), p.n_iters,
                                   p.reuse_paths, p.callback)
end

function solve(planner::RealTimeHeuristicSearch,
               domain::Domain, state::State, spec::Specification)
    # Simplify goal specification
    spec = simplify_goal(spec, domain, state)
    # Precompute heuristic information
    heuristic = planner.heuristic
    planner.heuristic = precomputed(heuristic, domain, state, spec)
    # Initialize then refine solution
    default = FunctionalVPolicy(planner.heuristic, domain, spec)
    sol = TabularVPolicy(domain, spec, default)
    if planner.reuse_paths
        sol = ReusableTreePolicy{typeof(state)}(sol)
        sol = solve!(sol, planner, domain, state, spec)
    else
        sol = solve!(sol, planner, domain, state, spec)
    end
    # Reset to original heuristic
    planner.heuristic = heuristic
    D, S = typeof(domain), typeof(state)
    return sol::Union{TabularVPolicy{D}, ReusableTreePolicy{S}}
end

function solve!(sol::Union{<:TabularVPolicy, <:ReusableTreePolicy},
                planner::RealTimeHeuristicSearch,
                domain::Domain, state::State, spec::Specification)
    @unpack n_iters, heuristic = planner
    # Use previously computed policy values to guide search
    planner.heuristic = PolicyValueHeuristic(sol)
    # Run callback if provided
    if !isnothing(planner.callback)
        planner.callback(planner, sol, state, state, 0, nothing, -Inf, nothing)
    end
    # Wrap specification to check for nodes in the reusable tree
    tree_spec = sol isa ReusableTreePolicy ?
        ReusableTreeGoal(spec, sol.tree) : spec
    # Iteratively perform heuristic search followed by simulated execution
    init_state = state
    act = PDDL.no_op
    for i_iter in 1:n_iters
        # Restart if goal is reached
        if is_goal(spec, domain, state, act)
            act = PDDL.no_op
            state = init_state
        end
        state_id = hash(state)
        # Update values of neighboring states via heuristic search
        best_q, best_act = -Inf, missing
        for act in available(domain, state)
            next_state = transition(domain, state, act)
            get_value(sol, next_state) == -Inf && continue
            if is_goal(spec, domain, next_state, act) # Goal reached
                next_v = 0.0
                if !has_action_goal(spec)
                    set_value!(sol, next_state, 0.0)
                end
            else # Run heuristic search
                search_sol = solve(planner.planner, domain, next_state, tree_spec)
                update_values!(planner, sol, search_sol, tree_spec)
                next_v = get_value(sol, next_state)
            end
            r = get_reward(spec, domain, state, act, next_state)
            q = get_discount(spec) * next_v + r
            if q > best_q
                best_q = q
                best_act = act
            end
            # Run callback if provided
            if !isnothing(planner.callback)
                planner.callback(planner, sol, init_state, state,
                                 i_iter, act, q, best_act)
            end
        end
        # Set value of current state to best of neighboring values
        set_value!(sol, state_id, best_q)
        # Run callback if provided
        if !isnothing(planner.callback)
            planner.callback(planner, sol, init_state, state,
                             i_iter, nothing, best_q, best_act)
        end
        # Follow policy one step forward if possible
        if !ismissing(best_act)
            act = best_act
            state = transition(domain, state, act)
        else
            act = PDDL.no_op
            state = init_state
        end
    end
    # Reset to original heuristic
    planner.heuristic = heuristic
    return sol
end

function update_values!(
    planner::RealTimeHeuristicSearch,
    policy::Union{<:TabularVPolicy, <:ReusableTreePolicy},
    search_sol::PathSearchSolution,
    spec::Specification
)
    @unpack h_mult = planner
    @unpack trajectory, search_tree, search_frontier = search_sol
    if search_sol.status == :failure
        # If search failed, then no nodes in the search tree can reach the goal
        for (node_id, _) in search_tree
            set_value!(policy, node_id, -Inf)
        end
    else
        # Get value of terminal node from search frontier
        terminal_id, (_, terminal_h_val, _) = dequeue_pair!(search_frontier)
        # Recompute f-value in case g_mult != 1.0
        terminal_path_cost = search_tree[terminal_id].path_cost
        terminal_h_val = h_mult * terminal_h_val
        terminal_f_val = terminal_path_cost + terminal_h_val
        # Set value of terminal node
        if search_sol.status != :success || !has_action_goal(spec)
            set_value!(policy, terminal_id, -terminal_h_val)
        end
        # Update values of all closed nodes in search tree
        for (node_id, node) in search_tree
            haskey(search_frontier, node_id) && continue
            est_cost_to_goal = terminal_f_val - node.path_cost
            set_value!(policy, node_id, -est_cost_to_goal)
        end
        # Insert path to terminal node into reusable tree
        if (policy isa ReusableTreePolicy && planner.reuse_paths &&
            search_sol.status == :success)
            insert_path!(policy, search_tree, terminal_id, terminal_f_val)
        end
    end
    return nothing
end

function refine!(sol::PolicySolution, planner::RealTimeHeuristicSearch,
                 domain::Domain, state::State, spec::Specification)
    return solve!(sol, planner, domain, state, spec)
end

function (cb::LoggerCallback)(
    planner::RealTimeHeuristicSearch,
    sol::PolicySolution, init_state::State, cur_state::State,
    n::Int, act, cur_v, best_act
)
    if n == 0 && get(cb.options, :log_header, true)
        @logmsg cb.loglevel "Running RTHS..."
        n_iters, max_nodes = planner.n_iters, planner.max_nodes
        @logmsg cb.loglevel "n_iters = $n_iters, max_nodes = $max_nodes"
        return nothing
    end
    log_period = get(cb.options, :log_period, 1)
    if n > 0 && n % log_period == 0 && !isnothing(act)
        act = write_pddl(act)
        @logmsg cb.loglevel "Iteration $n $act: value = $cur_v"
    end
    if n > 0 && n % log_period == 0 && isnothing(act)
        init_v = get_value(sol, init_state)
        best_act = write_pddl(best_act)
        @logmsg(cb.loglevel,
                "Iteration $n: best action = $best_act, " *
                "value = $cur_v, initial value = $init_v")
    end
    return nothing
end
