export RealTimeHeuristicSearch, RTHS

"""
    RealTimeHeuristicSearch(;
        heuristic::Heuristic = GoalCountHeuristic(),
        n_iters::Int = 1,
        max_nodes::Int = 50,
        update_method::Symbol = :costdiff,
        reuse_paths::Bool = true,
        kwargs...
    )

    RealTimeHeuristicSearch(
        planner::ForwardPlanner,
        n_iters::Int = 1,
        update_method::Symbol = :costdiff,
        reuse_paths::Bool = true,
    )

A real time heuristic search (`RTHS`) algorithm [1] similar to `RTDP`. Instead
of greedy rollouts, forward heuristic search is performed from each neighbor of
the current state (up to `max_nodes`), and value estimates are updated for all
states in the interiors of the resulting search trees. A simulated action
is then taken to the highest-value neighbor. This repeats for `n_iters`, with
future searches using the updated value estimates as more informed heuristics.

If `reuse_paths` is set to `true`, a [`ReusableTreePolicy`](@ref) is returned,
otherwise a [`TabularVPolicy`](@ref) is returned. While this planner returns a
policy, it expects a deterministic domain as input.

# Arguments

- `planner`: The [`ForwardPlanner`](@ref) used for lookahead heuristic search.
  Any keyword arguments accepted by [`ForwardPlanner`](@ref) are also keyword
  arguments for `RTHS` (e.g. `heuristic`, `max_nodes`, etc.).

- `n_iters`: The number of iterations to perform (default: 1).

- `update_method`: Method used to update value estimates, either via 
  cost differencing from the terminal node (`:costdiff`), or via Dijkstra's
  algorithm (`:dijkstra`).

- `reuse_paths`: If set to `true` (the default), then every time a path to the
  goal is found, it is stored in a reusable tree data structure. Following
  Tree Adaptive A* [2], subsequent searches will terminate once a path in
  the tree is encoutered, reducing the number of node expansions. Requires a
  consistent `heuristic` to preserve path optimality.

# Update Methods

Setting the `update_method` keyword argument controls how value estimates
``V(s)`` (or equivalently, cost-to-goal estimates ``h(s) = -V(s)``) are updated:

- `:costdiff` (default): Value estimates are updated by cost differencing from
  the terminal node ``t``. For each node ``s`` in the search tree's interior, we
  estimate the cost-to-goal ``h(s)`` by adding a lower bound on the cost from
  ``s`` to ``t`` to the cost-to-goal of ``t``:
  ```math
  h(s) = h(t) + (c(r, t) - c(r, s))
  ```
  where ``c(r, s)`` is the cost from the root node ``r`` to ``s``. This is
  the update used by Real-Time Adaptive A* [3]. Takes ``O(N)`` time, where ``N``
  is the size of the tree's interior.

- `:dijkstra`: Value estimates are backpropagated from the search frontier via
  Dijkstra's algorithm. For each node ``s`` in tree's interior, we update the
  cost-to-goal ``h(s)`` by minimizing over all paths to the frontier:
  ```math
  h(s) = \\min_{t \\in F} h(t) + c(s, t)
  ```
  This is the update rule by LSS-LRTA* [4], which produces more informed
  value estimates than `:costdiff`, but takes ``O(NB \\log NB)`` time. ``N`` is
  the size of the tree's interior and ``B`` is the maximal branching factor. 
  The `save_parents` keyword for [`ForwardPlanner`](@ref) defaults to `true`
  when this method is used.

Both of these update methods require a `heuristic` that is initially consistent
for the updated value estimates to remain consistent.

[1] R. E. Korf, "Real-Time Heuristic Search," Artificial Intelligence, vol. 42,
no. 2, pp. 189–211, Mar. 1990, <https://doi.org/10.1016/0004-3702(90)90054-4>.

[2] C. Hernández, X. Sun, S. Koenig, and P. Meseguer, "Tree Adaptive A*," 
AAMAS (2011), pp. 123–130. <https://dl.acm.org/doi/abs/10.5555/2030470.2030488>.

[3] S. Koenig and M. Likhachev, "Real-Time Adaptive A*," AAMAS (2006),
pp. 281–288. <https://doi.org/10.1145/1160633.1160682>.

[4] S. Koenig and X. Sun, "Comparing real-time and incremental heuristic
search for real-time situated agents", AAMAS (2009), pp. 313–341.
"""
@auto_hash_equals mutable struct RealTimeHeuristicSearch{
    P <: ForwardPlanner
} <: Planner
    planner::P
    n_iters::Int
    update_method::Symbol
    reuse_paths::Bool
    callback::Union{Nothing, Function}
end

const RTHS = RealTimeHeuristicSearch
@doc (@doc RealTimeHeuristicSearch) RTHS

function RealTimeHeuristicSearch(
    planner::ForwardPlanner;
    n_iters::Int = 1,
    update_method::Symbol = :costdiff,
    reuse_paths::Bool = true,
    verbose::Bool = false,
    callback = verbose ? LoggerCallback() : nothing,
)
    planner = copy(planner)
    planner.save_search = true
    if update_method == :dijkstra
        planner.save_parents = true
    end
    return RealTimeHeuristicSearch(planner, n_iters, update_method,
                                   reuse_paths, callback)
end

function RealTimeHeuristicSearch(;
    n_iters::Int = 1,
    max_nodes::Int = 50,
    update_method::Symbol = :costdiff,
    reuse_paths::Bool = true,
    verbose::Bool = false,
    callback = verbose ? LoggerCallback() : nothing,
    kwargs...
)
    planner = ForwardPlanner(;
        max_nodes = max_nodes, save_search = true, 
        save_parents = update_method == :dijkstra, kwargs...
    )
    return RealTimeHeuristicSearch(planner, n_iters, update_method, 
                                   reuse_paths, callback)
end

function RealTimeHeuristicSearch(
    heuristic::Heuristic;
    n_iters::Int = 1,
    max_nodes::Int = 50,
    update_method::Symbol = :costdiff,
    reuse_paths::Bool = true,
    verbose::Bool = false,
    callback = verbose ? LoggerCallback() : nothing,
    kwargs...
)
    planner = ForwardPlanner(;
        heuristic=heuristic, max_nodes=max_nodes, save_search=true,
        save_parents = update_method == :dijkstra, kwargs...
    )
    return RealTimeHeuristicSearch(planner, n_iters, update_method,
                                   reuse_paths, callback)
end

function Base.getproperty(planner::RealTimeHeuristicSearch, name::Symbol)
    name in (:planner, :n_iters, :update_method, :reuse_paths, :callback) ?
        getfield(planner, name) : getproperty(planner.planner, name)
end

function Base.setproperty!(planner::RealTimeHeuristicSearch, name::Symbol, val)
    name in (:planner, :n_iters, :update_method, :reuse_paths, :callback) ?
        setfield!(planner, name, val) : setproperty!(planner.planner, name, val)
end

function Base.copy(p::RealTimeHeuristicSearch)
    return RealTimeHeuristicSearch(copy(p.planner), p.n_iters, p.update_method,
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
    h = PruningHeuristic(PolicyValueHeuristic(sol), heuristic)
    planner.heuristic = h
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
                search_sol = solve(planner.planner, domain,
                                   next_state, tree_spec)
                update_values!(planner, sol, search_sol, domain, spec)
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
    domain::Domain,
    spec::Specification
)
    if search_sol.status == :failure
        # Set all values to -Inf since none can reach the goal
        for (node_id, _) in search_sol.search_tree
            set_value!(policy, node_id, -Inf)
        end
    elseif planner.update_method == :costdiff
        # Update values via cost differencing from the terminal node
        update_values_costdiff!(planner, policy, search_sol, domain, spec)
    elseif planner.update_method == :dijkstra
        # Update values via Dijkstra's algorithm from search frontier
        update_values_dijkstra!(planner, policy, search_sol, domain, spec)
    end
    # Insert path to the terminal node into reusable tree
    if (policy isa ReusableTreePolicy && planner.reuse_paths &&
        search_sol.status == :success)
        node_id, (_, h_val, _) = peek(search_sol.search_frontier)
        insert_path!(policy, search_sol.search_tree, node_id, h_val)
    end
    return nothing
end

"Cost-differencing update used by RTAA* (Koenig & Likhachev, 2006)."
function update_values_costdiff!(
    planner::RealTimeHeuristicSearch,
    policy::Union{<:TabularVPolicy, <:ReusableTreePolicy},
    search_sol::PathSearchSolution,
    domain::Domain,
    spec::Specification
)
    @unpack h_mult = planner
    @unpack trajectory, search_tree, search_frontier = search_sol
    # Get value of terminal node from search frontier
    terminal_id, (_, terminal_h_val, _) = peek(search_frontier)
    # Recompute f-value in case g_mult != 1.0
    terminal_path_cost = search_tree[terminal_id].path_cost
    terminal_f_val = terminal_path_cost + terminal_h_val
    # Set value of terminal node
    if !has_action_goal(spec) || search_sol.status != :success
        set_value!(policy, terminal_id, -terminal_h_val)
    end
    # Update values of all closed nodes in search tree
    for (node_id, node) in search_tree
        haskey(search_frontier, node_id) && continue
        h_val = terminal_f_val - node.path_cost
        set_value!(policy, node_id, -h_val)
    end
    return nothing
end

"Dijkstra update used by LSS-LRTA* (Koenig, 2004; Koenig & Sun, 2009)."
function update_values_dijkstra!(
    planner::RealTimeHeuristicSearch,
    policy::Union{<:TabularVPolicy, <:ReusableTreePolicy},
    search_sol::PathSearchSolution,
    domain::Domain,
    spec::Specification
)
    @unpack h_mult = planner
    @unpack trajectory, search_tree, search_frontier = search_sol
    # Construct priority queue of nodes ordered by h-values
    queue = PriorityQueue{UInt,Float32}()
    for node_id in keys(search_tree)
        h_val = haskey(search_frontier, node_id) ?
            search_frontier[node_id][2] : Inf32
        enqueue!(queue, node_id, h_val)
    end
    # Update values of all nodes in the search tree's interior
    visited = Set{UInt}()
    while !isempty(queue)
        node_id, h_val = dequeue_pair!(queue)
        node = search_tree[node_id]
        # Iterate over parents
        parent_ref = node.parent
        while !isnothing(parent_ref)
            parent_id = parent_ref.id
            parent_act = parent_ref.action
            parent_ref = parent_ref.next
            # Skip if parent node already has a lower h-value
            parent_h_val = haskey(queue, parent_id) ?
                queue[parent_id] : (-get_value(policy, parent_id) |> Float32)
            parent_h_val > h_val || continue
            parent = search_tree[parent_id]
            act_cost = get_cost(spec, domain, parent.state,
                                parent_act, node.state)
            # Skip if parent node's h-value can't be decreased
            parent_h_val > h_val + act_cost || continue
            parent_h_val = (h_val + act_cost) |> Float32
            # Update parent's h-value and insert into priority queue
            set_value!(policy, parent_id, -parent_h_val)
            queue[parent_id] = parent_h_val
            push!(visited, parent_id)
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
    if n > 0 && n % log_period == 0 && !isnothing(act) && !ismissing(act)
        act = write_pddl(act)
        @logmsg cb.loglevel "Iteration $n $act: value = $cur_v"
    end
    if n > 0 && n % log_period == 0 && isnothing(best_act) && !ismissing(best_act)
        init_v = get_value(sol, init_state)
        best_act = write_pddl(best_act)
        @logmsg(cb.loglevel,
                "Iteration $n: best action = $best_act, " *
                "value = $cur_v, initial value = $init_v")
    end
    return nothing
end
