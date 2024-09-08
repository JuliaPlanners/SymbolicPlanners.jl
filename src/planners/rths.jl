export RealTimeHeuristicSearch, RTHS

"""
    RealTimeHeuristicSearch(;
        heuristic::Heuristic = GoalCountHeuristic(),
        n_iters::Int = 1,
        max_nodes::Int = 50,
        update_method::Symbol = :dijkstra,
        search_neighbors::Symbol = :unexpanded,
        save_search::Bool = true,
        reuse_search::Bool = false,
        reuse_paths::Bool = true,
        kwargs...
    )

    RealTimeHeuristicSearch(
        planner::ForwardPlanner,
        n_iters::Int = 1,
        update_method::Symbol = :dijkstra,
        search_neighbors::Symbol = :unexpanded,
        reuse_search::Bool = true,
        reuse_paths::Bool = true,
    )

A real time heuristic search (`RTHS`) algorithm [1] similar to `RTDP`. Instead
of greedy rollouts, forward heuristic search is performed from the current state
(up to `max_nodes`), and value estimates are updated for all states in the
interior of the search tree. Search may also be performed from neighboring
states by configuring `search_neighbors`. A simulated action is then taken to
the best neighboring state. This process repeats for `n_iters`, with future
searches using the updated value function as a more informed heuristic.

Returns a [`ReusableTreePolicy`](@ref), containing a [`TabularVPolicy`](@ref)
of state value estimates, the most recent [`PathSearchSolution`](@ref) produced
by forward search (including a search tree if `save_search` is `true`), and a 
reusable tree of goal paths if `reuse_paths` is `true`.

# Arguments

- `planner`: The [`ForwardPlanner`](@ref) used for lookahead heuristic search.
  Any keyword arguments accepted by [`ForwardPlanner`](@ref) are also keyword
  arguments for `RTHS` (e.g. `heuristic`, `max_nodes`, `save_search`, etc.).

- `n_iters`: Number of iterations to perform. In each iteration,  search is
  followed by a simulated action to the best neighboring state. If a goal or
  dead end is reached, we restart from the initial state.

- `update_method`: Method used to update value estimates, either via 
  cost differencing from the terminal node (`:costdiff`), or via Dijkstra's
  algorithm (`:dijkstra`).

- `search_neighbors`: Controls whether search is additionally performed from all
  neighbors of the current state (`:all`), from neighbors unexpanded by the
  initial search (`:unexpanded`), or none of them (`:none`).

- `reuse_search`: If `true`, then previous search solutions are reused by
  subsequent searches in future iterations or calls to [`refine!`](@ref).
  The latter requires `save_search` to be `true`. Search solutions are reused
  via the `:reroot` refinement method for [`ForwardPlanner`](@ref).

- `reuse_paths`: If `true`, then every time a path to the goal is found, it is
  stored in a reusable tree of goal paths, as in Tree Adaptive A* [2]. Future
  searches will terminate once a previous path is encountered, reducing the cost
  of search. A consistent `heuristic` is required to ensure path optimality.

# Update Methods

Setting the `update_method` keyword argument controls how value estimates
``V(s)`` (or equivalently, cost-to-goal estimates ``h(s) = -V(s)``) are updated:

- `:costdiff`: Value estimates are updated by cost differencing from
  the terminal node ``t``. For each node ``s`` in the search tree's interior, we
  estimate the cost-to-goal ``h(s)`` by adding a lower bound on the cost from
  ``s`` to ``t`` to the cost-to-goal of ``t``:
  ```math
  h(s) = h(t) + (c(r, t) - c(r, s))
  ```
  where ``c(r, s)`` is the cost from the root node ``r`` to ``s``. This is
  the update used by Real-Time Adaptive A* [3]. Takes ``O(N)`` time, where ``N``
  is the size of the tree's interior.

- `:dijkstra` : Value estimates are backpropagated from the search frontier via
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
<https://dl.acm.org/doi/10.5555/1018410.1018838>.
"""
@auto_hash_equals mutable struct RealTimeHeuristicSearch{
    P <: ForwardPlanner
} <: Planner
    planner::P
    n_iters::Int
    update_method::Symbol
    search_neighbors::Symbol
    reuse_search::Bool
    reuse_paths::Bool
    callback::Union{Nothing, Function}
end

const RTHS = RealTimeHeuristicSearch
@doc (@doc RealTimeHeuristicSearch) RTHS

function RealTimeHeuristicSearch(
    planner::ForwardPlanner;
    n_iters::Int = 1,
    update_method::Symbol = :dijkstra,
    search_neighbors::Symbol = :unexpanded,
    reuse_search::Bool = false,
    reuse_paths::Bool = true,
    verbose::Bool = false,
    callback = verbose ? LoggerCallback() : nothing,
)
    planner = copy(planner)
    # Ensure that the search solution can be re-rooted
    if reuse_search == true
        planner.refine_method = :reroot
        planner.save_parents = true
        planner.save_children = true
        planner.save_search = true
    else
        planner.refine_method = :restart
    end
    # Ensure all search node parents are saved for Dijkstra updating
    if update_method == :dijkstra
        planner.save_parents = true
    end
    return RealTimeHeuristicSearch(
        planner, n_iters, update_method, search_neighbors,
        reuse_search, reuse_paths, callback
    )
end

function RealTimeHeuristicSearch(;
    n_iters::Int = 1,
    max_nodes::Int = 50,
    update_method::Symbol = :dijkstra,
    search_neighbors::Symbol = :unexpanded,
    save_search::Bool = true,
    reuse_search::Bool = false,
    reuse_paths::Bool = true,
    verbose::Bool = false,
    callback = verbose ? LoggerCallback() : nothing,
    kwargs...
)
    planner = ForwardPlanner(;
        max_nodes = max_nodes, save_search = save_search, kwargs...
    )
    # Ensure that the search solution can be re-rooted
    if reuse_search == true
        planner.refine_method = :reroot
        planner.save_parents = true
        planner.save_children = true
        planner.save_search = true
    else
        planner.refine_method = :restart
    end
    # Ensure all search node parents are saved for Dijkstra updating
    if update_method == :dijkstra
        planner.save_parents = true
    end
    return RealTimeHeuristicSearch(
        planner, n_iters, update_method, search_neighbors,
        reuse_search, reuse_paths, callback
    )
end

function RealTimeHeuristicSearch(heuristic::Heuristic; kwargs...)
    return RealTimeHeuristicSearch(; heuristic = heuristic, kwargs...)
end

function Base.getproperty(planner::P, name::Symbol) where {P <: RTHS}
    hasfield(P, name) ?
        getfield(planner, name) : getproperty(planner.planner, name)
end

function Base.setproperty!(planner::P, name::Symbol, val) where {P <: RTHS}
    hasfield(P, name) ?
        setfield!(planner, name, val) : setproperty!(planner.planner, name, val)
end

function Base.hasproperty(planner::P, name::Symbol) where {P <: RTHS}
    hasfield(P, name) || hasproperty(planner.planner, name)
end

function Base.copy(p::RealTimeHeuristicSearch)
    return RealTimeHeuristicSearch(
        copy(p.planner), p.n_iters, p.update_method, p.search_neighbors,
        p.reuse_search, p.reuse_paths, p.callback
    )
end

function solve(planner::RealTimeHeuristicSearch,
               domain::Domain, state::State, spec::Specification)
    # Simplify goal specification
    spec = simplify_goal(spec, domain, state)
    # Precompute heuristic information
    planner = copy(planner)
    planner.heuristic = precomputed(planner.heuristic, domain, state, spec)
    # Initialize then refine solution
    default = HeuristicVPolicy(planner.heuristic, domain, spec)
    value_policy = TabularVPolicy(domain, spec, default)
    search_sol = init_sol(planner.planner, planner.heuristic,
                          domain, state, spec)
    sol = ReusableTreePolicy{typeof(state)}(value_policy, search_sol)
    sol = solve!(sol, planner, domain, state, spec)
    return sol
end

function solve!(sol::ReusableTreePolicy, planner::RealTimeHeuristicSearch,
                domain::Domain, state::State, spec::Specification)
    @unpack n_iters, heuristic, search_neighbors = planner
    @unpack save_search, reuse_search, reuse_paths, callback = planner
    planner = copy(planner)
    # Use previously computed policy values to guide search
    planner.heuristic = PruningHeuristic(PolicyValueHeuristic(sol), heuristic)
    # Ensure that search planner returns a search tree in its solution
    planner.save_search = true
    search_planner = planner.planner
    # Run callback if provided
    if !isnothing(callback)
        callback(planner, sol, state, state, 0, nothing, -Inf, nothing)
    end
    # Iteratively perform heuristic search followed by simulated execution
    init_state = state
    act = PDDL.no_op
    for i in 1:n_iters
        # Restart if goal is reached
        if is_goal(spec, domain, state, act)
            act = PDDL.no_op
            state = init_state
        end
        # Reprioritize nodes on the search frontier based on updated values
        if reuse_search && is_expanded(state, sol.search_sol)
            reorder_queue!(sol.search_sol, planner, sol)
        end
        # Run search from current state and update values
        if reuse_paths
            tree_spec = ReusableTreeGoal(spec, sol.goal_tree)
            refine!(sol.search_sol, search_planner, domain, state, tree_spec)
        else
            refine!(sol.search_sol, search_planner, domain, state, spec)
        end
        update_values!(sol, planner, domain, spec, sol.search_sol)
        # Run search from neighboring states
        if search_neighbors != :none && reuse_paths
            tree_spec = ReusableTreeGoal(spec, sol.goal_tree)
            search_neighbors!(sol, planner, domain, state, spec, tree_spec, i)
        elseif search_neighbors != :none
            search_neighbors!(sol, planner, domain, state, spec, spec, i)
        end
        # Run callback if provided
        if !isnothing(callback)
            callback(planner, sol, init_state, state, i, nothing,
                     get_value(sol, state), best_action(sol, state))
        end
        # Follow policy one step forward if possible
        act = best_action(sol, state)
        act = ismissing(act) ? PDDL.no_op : act
        state = ismissing(act) ? init_state : transition(domain, state, act)
    end
    # Empty search tree and queue if `save_search` was originally false
    if !save_search
        empty!(sol.search_sol.search_tree)
        empty!(sol.search_sol.search_frontier)
        empty!(sol.search_sol.search_order)
        sol.search_sol.expanded = -1
    end
    return sol
end

function search_neighbors!(
    sol::ReusableTreePolicy, planner::RealTimeHeuristicSearch,
    domain::Domain, state::State, spec::Specification,
    tree_spec::Specification = spec, i_iter::Int = 1
)
    @unpack reuse_search, search_neighbors, callback = planner
    # Avoid resetting node count in neighbor refinement steps
    search_planner = copy(planner.planner)
    search_planner.reset_node_count = false
    # Run search from neighboring states
    for act in available(domain, state)
        next_state = transition(domain, state, act)
        get_value(sol, next_state) == -Inf && continue
        next_expanded = is_expanded(next_state, sol.search_sol)
        search_sol = nothing
        if is_goal(spec, domain, next_state, act)
            # Goal reached, set value of neighboring state to 0
            !has_action_goal(spec) && set_value!(sol, next_state, 0.0)
        elseif search_neighbors == :all && reuse_search && next_expanded
            # Refine search starting from neighboring state
            search_sol = copy(sol.search_sol)
            reorder_queue!(search_sol, planner, sol)
            refine!(search_sol, search_planner, domain, next_state, tree_spec)
            update_values!(sol, planner, domain, spec, search_sol)
        elseif search_neighbors == :all
            # Run new search from neighboring state
            search_sol = solve(search_planner, domain, next_state, tree_spec)
            update_values!(sol, planner, domain, spec, search_sol)
        elseif search_neighbors == :unexpanded && !next_expanded
            # Run new search from unexpanded neighboring state
            search_sol = solve(search_planner, domain, next_state, tree_spec)
            update_values!(sol, planner, domain, spec, search_sol)
        end
        # Run callback if search was performed
        if !isnothing(callback) && !isnothing(search_sol)
            callback(planner, sol, nothing, state, i_iter,
                     act, get_value(sol, state, act), nothing)
        end
    end
    # Set value of current state to best of updated neighboring values
    best_q = -Inf
    for act in available(domain, state)
        best_q = max(best_q, get_value(sol, state, act))
    end
    set_value!(sol, state, best_q)
    return sol
end

function update_values!(
    policy::ReusableTreePolicy, planner::RealTimeHeuristicSearch,
    domain::Domain, spec::Specification, search_sol::PathSearchSolution
)
    if search_sol.status == :failure
        # Set all values to -Inf since none can reach the goal
        for (node_id, _) in search_sol.search_tree
            set_value!(policy, node_id, -Inf)
        end
    elseif planner.update_method == :costdiff
        # Update values via cost differencing from the terminal node
        update_values_costdiff!(policy, planner, domain, spec, search_sol)
    elseif planner.update_method == :dijkstra
        # Update values via Dijkstra's algorithm from search frontier
        update_values_dijkstra!(policy, planner, domain, spec, search_sol)
    end
    # Insert path to the terminal node into reusable tree
    if planner.reuse_paths && search_sol.status == :success
        node_id, (_, h_val, _) = peek(search_sol.search_frontier)
        insert_path!(policy, search_sol.search_tree, node_id, h_val)
    end
    return policy
end

"Cost-differencing update used by RTAA* (Koenig & Likhachev, 2006)."
function update_values_costdiff!(
    policy::ReusableTreePolicy, planner::RealTimeHeuristicSearch,
    domain::Domain, spec::Specification, search_sol::PathSearchSolution
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
    return policy
end

"Dijkstra update used by LSS-LRTA* (Koenig, 2004; Koenig & Sun, 2009)."
function update_values_dijkstra!(
    policy::ReusableTreePolicy, planner::RealTimeHeuristicSearch,
    domain::Domain, spec::Specification, search_sol::PathSearchSolution
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
            # Skip self-edges / edges without actions
            isnothing(parent_act) && continue
            parent_id == node_id && continue
            # Skip if parent node already has a lower h-value
            parent_h_val = haskey(queue, parent_id) ? queue[parent_id] :
                (-get_value(policy, parent_id, -Inf32) |> Float32)
            parent_h_val > h_val || continue
            # Skip parents that were deleted by rerooting etc.
            parent = get(search_tree, parent_id, nothing)
            isnothing(parent) && continue
            # Skip if parent node's h-value can't be decreased
            act_cost = get_cost(spec, domain, parent.state,
                                parent_act, node.state)
            parent_h_val > h_val + act_cost || continue
            parent_h_val = (h_val + act_cost) |> Float32
            # Update parent's h-value and insert into priority queue
            set_value!(policy, parent_id, -parent_h_val)
            queue[parent_id] = parent_h_val
            push!(visited, parent_id)
        end
    end
    return policy
end

function reorder_queue!(
    search_sol::PathSearchSolution, planner::RealTimeHeuristicSearch,
    policy::ReusableTreePolicy,
)
    @unpack h_mult = planner
    queue = search_sol.search_frontier
    for (node_id, priority) in queue
        f_val, h_val, n = priority
        !has_cached_value(policy, node_id) && continue
        new_h_val::Float32 = -get_value(policy, node_id)
        new_h_val == h_val && continue
        new_f_val::Float32 = f_val + h_mult * (new_h_val - h_val) 
        new_priority = (new_f_val, new_h_val, n)
        queue[node_id] = new_priority
    end
    return search_sol
end

function refine!(sol::PolicySolution, planner::RealTimeHeuristicSearch,
                 domain::Domain, state::State, spec::Specification)
    return solve!(sol, planner, domain, state, spec)
end

function (cb::LoggerCallback)(
    planner::RealTimeHeuristicSearch,
    sol::PolicySolution, init_state::Union{Nothing, State}, cur_state::State,
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
        act_str = write_pddl(act)
        @logmsg cb.loglevel "Iteration $n $act_str: value = $cur_v"
    end
    if n > 0 && n % log_period == 0 && !isnothing(best_act)
        init_v = get_value(sol, init_state)
        best_act_str = ismissing(best_act) ? "missing" : write_pddl(best_act)
        @logmsg(cb.loglevel,
                "Iteration $n: best action = $best_act_str, " *
                "value = $cur_v, initial state value = $init_v")
    end
    return nothing
end
