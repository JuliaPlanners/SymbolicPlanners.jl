export RealTimeHeuristicSearch, RTHS

"""
    RealTimeHeuristicSearch(;
        heuristic::Heuristic = GoalCountHeuristic(),
        n_iters::Int = 10,
        max_nodes::Int = 50,
        kwargs...
    )

    RealTimeHeuristicSearch(
        planner::ForwardPlanner,
        n_iters::Int
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
backpropagating the value estimates of all frontier nodes (including goal nodes)
to their parents. That is, for each ancestor ``a`` of a frontier node ``n`` in
the search tree, its updated value estimate is:

```math
V(a) = -[f(n) - g(a)] = -[g(n) + h(n) - g(a)] = -[(g(n) - g(a)) + h(n)]
```

where ``g(n)`` is the path cost from the root of the search tree to ``n``, 
``h(n)`` is the heuristic goal-distance estimate for ``n``, and
``f(n) = g(n) + h(n)``, the priority value for the frontier node.

Intuitively, the updated value of ``V(a)`` is the (negative) estimated cost
from ``a`` to the goal, computed by summing the distance from ``a`` to ``n``
with the estimated distance of ``n`` to the goal. In cases where frontier nodes
share the same ancestor, frontier nodes with lower ``f`` values take precedence.
This update rule is a variant of Learning Real Time A* (LRTA) [1], similar 
to the update rule used by Real-Time Adaptive A* (RTAA) [2] because it updates 
both the root node and other nodes in the search tree.

Returns a [`TabularVPolicy`](@ref), which stores the value estimates for each
encountered state. Note that while this planner returns a policy, it expects
a deterministic domain as input.

[1] R. E. Korf, "Real-Time Heuristic Search," Artificial Intelligence, vol. 42,
no. 2, pp. 189–211, Mar. 1990, <https://doi.org/10.1016/0004-3702(90)90054-4>.

[2] S. Koenig and M. Likhachev, “Real-Time Adaptive A*,” AAMAS (2006), 
pp. 281–288. <https://doi.org/10.1145/1160633.1160682>.
"""
mutable struct RealTimeHeuristicSearch <: Planner
    planner::ForwardPlanner
    n_iters::Int
end

@auto_hash RealTimeHeuristicSearch
@auto_equals RealTimeHeuristicSearch

const RTHS = RealTimeHeuristicSearch

function RealTimeHeuristicSearch(planner::ForwardPlanner)
    planner = copy(planner)
    planner.save_search = true
    return RealTimeHeuristicSearch(planner, 1)
end

function RealTimeHeuristicSearch(;
    n_iters::Int = 10, max_nodes::Int = 50, kwargs...
)
    planner = ForwardPlanner(;max_nodes=max_nodes, save_search=true, kwargs...)
    return RealTimeHeuristicSearch(planner, n_iters)
end

function RealTimeHeuristicSearch(
    heuristic::Heuristic;
    n_iters::Int = 10, max_nodes::Int = 50, kwargs...
)
    planner = ForwardPlanner(;heuristic=heuristic, max_nodes=max_nodes,
                              save_search=true, kwargs...)
    return RealTimeHeuristicSearch(planner, n_iters)
end

function Base.getproperty(planner::RealTimeHeuristicSearch, name::Symbol)
    return name in (:planner, :n_iters) ?
        getfield(planner, name) : getproperty(planner.planner, name)
end

function Base.setproperty!(planner::RealTimeHeuristicSearch, name::Symbol, val)
    return name in (:planner, :n_iters) ?
        setfield!(planner, name, val) : setproperty!(planner.planner, name, val)
end

function Base.copy(p::RealTimeHeuristicSearch)
    return RealTimeHeuristicSearch(copy(p.planner), p.n_iters)
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
    sol = solve!(sol, planner, domain, state, spec)
    # Reset to original heuristic
    planner.heuristic = heuristic
    return sol
end

function solve!(sol::TabularVPolicy, planner::RealTimeHeuristicSearch,
                domain::Domain, state::State, spec::Specification)
    @unpack n_iters, heuristic = planner
    # Use previously computed policy values to guide search 
    planner.heuristic = PolicyValueHeuristic(sol)
    # Iteratively perform heuristic search followed by simulated execution
    init_state = state
    for _ in 1:n_iters
        # Restart if goal is reached
        state = is_goal(spec, domain, state) ? init_state : state
        state_id = hash(state)
        # Update values of neighboring states via heuristic search
        best_q, best_act = -Inf, nothing
        for act in available(domain, state)
            next_state = transition(domain, state, act)
            next_id = hash(next_state)
            search_sol = solve(planner.planner, domain, next_state, spec)
            update_values!(planner, sol, search_sol, domain, spec)
            r = get_reward(spec, domain, state, act, next_state)
            q = get_discount(spec) * sol.V[next_id] + r
            if q > best_q
                best_q = q
                best_act = act
            end
        end
        # Set value of current state to best of neighboring values
        sol.V[state_id] = best_q
        # Follow policy one step forward if possible
        state = isnothing(best_act) ?
            init_state : transition(domain, state, best_act)
    end
    # Reset to original heuristic
    planner.heuristic = heuristic
    return sol
end

function update_values!(planner::RealTimeHeuristicSearch,
                        policy::TabularVPolicy, search_sol::PathSearchSolution,
                        domain::Domain, spec::Specification)
    @unpack h_mult, heuristic = planner
    @unpack trajectory, search_tree = search_sol
    # Back-propagate values from frontier nodes in increasing priority order
    visited = Set{UInt}()
    queue = copy(search_sol.search_frontier)
    while !isempty(queue)
        node_id, (frontier_f_val, _, _) = dequeue_pair!(queue)
        # Iterate until root or a node that is already visited
        while !(node_id in visited)
            push!(visited, node_id)
            node = search_tree[node_id]
            est_cost_to_goal = frontier_f_val - node.path_cost
            policy.V[node_id] = -est_cost_to_goal
            if node.parent_id === nothing break end
            node_id = node.parent_id
        end
    end
    return nothing
end

function refine!(sol::PolicySolution, planner::RealTimeHeuristicSearch,
                 domain::Domain, state::State, spec::Specification)
    return solve!(sol, planner, domain, state, spec)
end
