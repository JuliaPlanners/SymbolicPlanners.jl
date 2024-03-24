export ForwardPlanner, BestFirstPlanner, UniformCostPlanner, GreedyPlanner
export AStarPlanner, WeightedAStarPlanner
export ProbForwardPlanner, ProbAStarPlanner

"""
    ForwardPlanner(;
        heuristic::Heuristic = GoalCountHeuristic(),
        search_noise::Union{Nothing,Float64} = nothing,
        g_mult::Float32 = 1.0f0,
        h_mult::Float32 = 1.0f0,
        max_nodes::Int = typemax(Int),
        max_time::Float64 = Inf,
        fail_fast::Bool = false,
        refine_method::Symbol = :continue,
        reset_node_count::Bool = true,
        save_search::Bool = true,
        save_search_order::Bool = true,
        save_parents::Bool = false,
        save_children::Bool = false,
        verbose::Bool = false,
        callback = verbose ? LoggerCallback() : nothing
    )

Forward best-first search planner, which encompasses uniform-cost search,
greedy search, and A* search. Each node ``n`` is expanded in order of increasing
priority ``f(n)``, defined as:

```math
f(n) = g_\\text{mult} \\cdot g(n) + h_\\text{mult} \\cdot h(n)
```

where ``g(n)`` is the path cost from the initial state to ``n``, and ``h(n)``
is the heuristic's goal distance estimate.

Returns a [`PathSearchSolution`](@ref) if the goal is achieved, containing a
plan that reaches the goal node, and `status` set to `:success`. If the node
or time budget runs out, the solution will instead contain a partial plan to
the last node selected for expansion, with `status` set to `:max_nodes` or
`:max_time` accordingly.

If `save_search` is true, the returned solution will contain the search tree
and frontier so far. If `save_search` is true and the search space is exhausted
return a `NullSolution` with `status` set to `:failure`.

# Arguments

$(FIELDS)

# Refinement Methods

Setting the `refine_method` keyword argument controls the behavior of
[`refine!`](@ref) when called on a [`PathSearchSolution`](@ref):

- `:continue` (default): Continues the search by expanding the search tree
  rooted at the original starting state. `save_search` will default to `true`
  if this method is used.

- `:reroot`: Reroots the search tree at the newly-provided starting
  state, then continues the search, as in Fringe-Retrieving A* [1].
  `save_search`, `save_parents`, and `save_children` will default to `true`
  if this method is used.

- `:restart`: Restarts the search from scratch, throwing away the 
  previous search tree and frontier. This is the only valid refinement method
  when `save_search` is `false`.

[1] X. Sun, W. Yeoh, and S. Koenig, “Generalized Fringe-Retrieving A*: Faster
moving target search on state lattices,” AAMAS (2010), pp. 1081-1088.
<https://dl.acm.org/doi/abs/10.5555/1838206.1838352>
"""
@kwdef mutable struct ForwardPlanner{T <: Union{Nothing, Float64}}  <: Planner
    "Search heuristic that estimates cost of a state to the goal."
    heuristic::Heuristic = GoalCountHeuristic()
    "Amount of Boltzmann search noise (`nothing` for deterministic search)."
    search_noise::T = nothing
    "Path cost multiplier when computing the ``f`` value of a search node."
    g_mult::Float32 = 1.0f0
    "Heuristic multiplier when computing the ``f`` value of a search node."
    h_mult::Float32 = 1.0f0
    "Maximum number of search nodes before termination."
    max_nodes::Int = typemax(Int)
    "Maximum time in seconds before planner times out."
    max_time::Float64 = Inf
    "Flag to terminate search if the heuristic estimates an infinite cost."
    fail_fast::Bool = false
    "Solution refinement method (one of `:continue`, `:reroot`, `:restart`)"
    refine_method::Symbol = :continue
    "Whether to reset the expanded node count before solution refinement."
    reset_node_count::Bool = true
    "Flag to save the search tree and frontier (needed for refinement)."
    save_search::Bool = refine_method != :restart
    "Flag to save the node expansion order in the solution."
    save_search_order::Bool = save_search
    "Flag to save all parent pointers in search tree (needed for rerooting)."
    save_parents::Bool = refine_method == :reroot
    "Flag to save all children pointers in search tree (needed for rerooting)."
    save_children::Bool = refine_method == :reroot
    "Flag to print debug information during search."
    verbose::Bool = false
    "Callback function for logging, etc."
    callback::Union{Nothing, Function} = verbose ? LoggerCallback() : nothing
end

@auto_hash ForwardPlanner
@auto_equals ForwardPlanner

ForwardPlanner(heuristic::Heuristic, search_noise::T, args...) where {T} =
    ForwardPlanner{T}(heuristic, search_noise, args...)

"""
$(SIGNATURES)

Best-first search planner (alias for [`ForwardPlanner`](@ref)).
"""
BestFirstPlanner(args...; kwargs...) =
    ForwardPlanner(args...; kwargs...)

"""
$(SIGNATURES)

Uniform-cost search. Nodes with the lowest path cost from the initial state
are expanded first (i.e. the search heuristic is not used).
"""
UniformCostPlanner(;kwargs...) =
    ForwardPlanner(;heuristic=NullHeuristic(), h_mult=0, kwargs...)

"""
$(SIGNATURES)

Greedy best-first search, with cycle checking. Nodes with the lowest heuristic
value are expanded first (i.e. the cost of reaching them from the initial state
is ignored).
"""
GreedyPlanner(heuristic::Heuristic; kwargs...) =
    ForwardPlanner(;heuristic=heuristic, g_mult=0, kwargs...)

"""
$(SIGNATURES)

A* search. Nodes with the lowest ``f`` value are expanded first. This is
guaranteed to produce a cost-optimal solution if the `heuristic` is admissible.
"""
AStarPlanner(heuristic::Heuristic; kwargs...) =
    ForwardPlanner(;heuristic=heuristic, kwargs...)

"""
$(SIGNATURES)

Weighted A* search, which multiplies the heuristic estimate by `h_mult`
when computing the ``f`` value of a node. Nodes with the lowest ``f`` value
are expanded first.
"""
WeightedAStarPlanner(heuristic::Heuristic, h_mult::Real; kwargs...) =
    ForwardPlanner(;heuristic=heuristic, h_mult=h_mult, kwargs...)

"""
    ProbForwardPlanner(;
        search_noise::Float64 = 1.0,
        kwargs...
    )

A probabilistic variant of forward best-first search. Instead of always
expanding the node with lowest ``f`` value in the search frontier, this samples
a node to expand according to Boltzmann distribution, where the ``f`` value of
a frontier node is treated as the unnormalized log probability of expansion.

The temperature for Boltzmann sampling is defined by `search_noise`. Higher
values lead to more random search, lower values lead to more deterministic
search.

Useful for simulating a diversity of potentially sub-optimal plans, especially
when paired with a limited `max_nodes` budget.

An alias for `ForwardPlanner{Float64}`. See [`ForwardPlanner`](@ref) for other
arguments.
"""
const ProbForwardPlanner = ForwardPlanner{Float64}

ProbForwardPlanner(;search_noise=1.0, kwargs...) =
    ForwardPlanner(;search_noise=search_noise, kwargs...)

"""
$(SIGNATURES)

A probabilistic variant of A* search. See [`ProbForwardPlanner`](@ref) for
how nodes are probabilistically expanded.
"""
ProbAStarPlanner(heuristic::Heuristic; search_noise=1.0, kwargs...) =
    ForwardPlanner(;heuristic=heuristic, search_noise=search_noise, kwargs...)

function Base.copy(p::ForwardPlanner)
    return ForwardPlanner(
        p.heuristic, p.search_noise, p.g_mult, p.h_mult,
        p.max_nodes, p.max_time, p.fail_fast, p.refine_method,
        p.reset_node_count, p.save_search, p.save_search_order,
        p.save_parents, p.save_children, p.verbose, p.callback
    )
end

function solve(planner::ForwardPlanner,
               domain::Domain, state::State, spec::Specification)
    @unpack heuristic, save_search = planner
    # Simplify goal specification
    spec = simplify_goal(spec, domain, state)
    # Precompute heuristic information
    precompute!(heuristic, domain, state, spec)
    # Initialize solution
    sol = init_sol(planner, heuristic, domain, state, spec)
    # Run the search
    sol = search!(sol, planner, heuristic, domain, spec)
    # Return solution
    if save_search
        return sol
    elseif sol.status == :failure
        return NullSolution(sol.status)
    else
        return PathSearchSolution(sol.status, sol.plan, sol.trajectory)
    end
end

function init_sol(planner::ForwardPlanner, heuristic::Heuristic,
                  domain::Domain, state::State, spec::Specification)
    # Initialize search tree and priority queue
    node_id = hash(state)
    node = PathNode(node_id, state, 0.0, LinkedNodeRef(node_id))
    search_tree = Dict(node_id => node)
    h_val::Float32 = compute(heuristic, domain, state, spec)
    priority = (planner.h_mult * h_val, h_val, 0)
    queue = PriorityQueue(node_id => priority)
    search_order = UInt[]
    sol = PathSearchSolution(:in_progress, Term[], Vector{typeof(state)}(),
                             0, search_tree, queue, search_order)
    return sol
end

function reinit_sol!(
    sol::PathSearchSolution{S, T}, 
    planner::ForwardPlanner, heuristic::Heuristic,
    domain::Domain, state::State, spec::Specification
) where {S, T <: PriorityQueue}
    search_tree, queue = sol.search_tree, sol.search_frontier
    sol.status = :in_progress
    # Empty containers
    empty!(sol.plan)
    empty!(sol.trajectory)
    empty!(sol.search_order)
    # Reinitialize search tree
    empty!(search_tree)
    node_id = hash(state)
    node = PathNode(node_id, state, 0.0, LinkedNodeRef(node_id))
    search_tree[node_id] = node
    # Reinitialize priority queue
    empty!(queue)
    h_val::Float32 = compute(heuristic, domain, state, spec)
    priority = (planner.h_mult * h_val, h_val, 0)
    queue[node_id] = priority
    return sol
end

function search!(sol::PathSearchSolution,
                 planner::ForwardPlanner, heuristic::Heuristic,
                 domain::Domain, spec::Specification)
    @unpack search_noise = planner
    start_time = time()
    queue, search_tree = sol.search_frontier, sol.search_tree
    while length(queue) > 0
        # Get state with lowest estimated cost to goal
        node_id, priority = isnothing(search_noise) ?
            peek(queue) : prob_peek(queue, search_noise)
        node = search_tree[node_id]
        # Check search termination criteria
        if is_goal(spec, domain, node.state, node.parent.action)
            sol.status = :success # Goal reached
        elseif sol.expanded >= planner.max_nodes
            sol.status = :max_nodes # Node budget reached
        elseif time() - start_time >= planner.max_time
            sol.status = :max_time # Time budget reached
        elseif planner.fail_fast && priority[1] == Inf
            sol.status = :failure # Search space exhausted
            break
        end
        if sol.status == :in_progress
            # Dequeue current node
            isnothing(search_noise) ? dequeue!(queue) : delete!(queue, node_id)
            # Expand current node
            expand!(planner, heuristic, node, search_tree, queue, domain, spec)
            sol.expanded += 1
            if planner.save_search && planner.save_search_order
                push!(sol.search_order, node_id)
            end
            if !isnothing(planner.callback)
                planner.callback(planner, sol, node_id, priority)
            end
        else # Reconstruct plan and return solution
            sol.plan, sol.trajectory = reconstruct(node_id, search_tree)
            if !isnothing(planner.callback)
                planner.callback(planner, sol, node_id, priority)
            end
            return sol
        end
    end
    if !isnothing(planner.callback)
        planner.callback(planner, sol, nothing, (Inf32, Inf32, 0))
    end
    sol.status = :failure
    return sol
end

function expand!(
    planner::ForwardPlanner, heuristic::Heuristic, node::PathNode{S},
    search_tree::Dict{UInt,PathNode{S}}, queue::PriorityQueue,
    domain::Domain, spec::Specification
) where {S <: State}
    @unpack g_mult, h_mult = planner
    state = node.state
    # Iterate over available actions, filtered by heuristic
    for act in filter_available(heuristic, domain, state, spec)
        # Execute action and trigger all post-action events
        next_state = transition(domain, state, act; check=false)
        next_id = hash(next_state)
        # Check if next state satisfies trajectory constraints
        if is_violated(spec, domain, state) continue end
        # Compute path cost
        act_cost = get_cost(spec, domain, state, act, next_state)
        path_cost = node.path_cost + act_cost
        # Check if action goal is reached
        is_action_goal = false
        if has_action_goal(spec) && is_goal(spec, domain, next_state, act)
            is_action_goal = true
            next_id = hash((next_state, act))
        end
        # Construct or retrieve child node
        next_node = get!(search_tree, next_id) do
            PathNode{S}(next_id, next_state, Inf32)
        end
        cost_diff = next_node.path_cost - path_cost
        if cost_diff > 0  # Update path costs if new path is shorter
            next_node.path_cost = path_cost
            # Update parent and child pointers
            if planner.save_parents
                next_node.parent = LinkedNodeRef(node.id, act, next_node.parent)
            else
                next_node.parent = LinkedNodeRef(node.id, act)
            end
            if planner.save_children
                node.child = LinkedNodeRef(next_id, nothing, node.child)
            end
            # Update estimated cost from next state to goal
            if !(next_id in keys(queue))
                h_val::Float32 = is_action_goal ?
                    0.0f0 : compute(heuristic, domain, next_state, spec)
                f_val::Float32 = g_mult * path_cost + h_mult * h_val
                priority = (f_val, h_val, length(search_tree))
                enqueue!(queue, next_id, priority)
            else
                f_val, h_val, n_nodes = queue[next_id]
                queue[next_id] = (f_val - cost_diff, h_val, n_nodes)
            end
        elseif planner.save_parents # Update parent pointers
            next_node.parent.next =
                LinkedNodeRef(node.id, act, next_node.parent.next)
        end
    end
end

function refine!(
    sol::PathSearchSolution{S, T}, planner::ForwardPlanner,
    domain::Domain, state::State, spec::Specification
) where {S, T <: PriorityQueue}
    @unpack heuristic, refine_method, reset_node_count = planner
    # Immmediately return if solution is already complete
    sol.status == :success && return sol
    sol.status == :failure && return sol
    sol.status = :in_progress
    # Resimplify goal specification and ensure heuristic is precomputed
    spec = simplify_goal(spec, domain, state)
    ensure_precomputed!(heuristic, domain, state, spec)
    # Decide between restarting, rerooting, or continuing the search
    if refine_method == :restart
        init_state = sol.trajectory[1]
        sol = reinit_sol!(sol, planner, heuristic, domain, init_state, spec)
    elseif refine_method == :reroot
        reroot!(sol, planner, heuristic, domain, state, spec)
    end
    planner.reset_node_count && (sol.expanded = 0)
    # Run search and return solution
    return search!(sol, planner, heuristic, domain, spec)
end

function reroot!(
    sol::PathSearchSolution{S}, planner::ForwardPlanner, heuristic::Heuristic,
    domain::Domain, state::S, spec::Specification
) where {S <: State}
    @unpack h_mult, g_mult = planner
    queue, search_tree = sol.search_frontier, sol.search_tree
    cb = planner.callback
    verbose = cb isa LoggerCallback
    verbose && @logmsg cb.loglevel "Rerooting search tree..."
    # Restart search if initial state is not in tree interior
    root_id = hash(state)
    if !haskey(search_tree, root_id) || haskey(queue, root_id) 
        return reinit_sol!(sol, planner, heuristic, domain, state, spec)
    end
    # Detach new root node from parents
    root_node = search_tree[root_id]
    root_node.parent = LinkedNodeRef(root_id)
    # Mark all nodes not rooted at the new root for deletion
    verbose && @logmsg cb.loglevel "Marking nodes for deletion..."
    prev_root_id = hash(sol.trajectory[1])
    deleted = Set{UInt}()
    del_queue = [prev_root_id]
    while !isempty(del_queue)
        del_id = pop!(del_queue)
        del_id == root_id && continue # Skip new root
        del_node = search_tree[del_id]
        push!(deleted, del_id)
        # Iterate over children
        child_ref = del_node.child
        del_node.child = nothing
        while !isnothing(child_ref)
            child_id = child_ref.id
            child_ref = child_ref.next
            child_id in deleted && continue # Skip if already marked
            child = get(search_tree, child_id, nothing)
            isnothing(child) && continue
            child.parent.id == del_id || continue # Consistency check
            push!(del_queue, child_id)
        end
    end
    # Delete nodes not on the frontier of the new search tree
    verbose && @logmsg cb.loglevel "Deleting or reparenting marked nodes..."
    adopters = Set{UInt}()
    n_adopted = 0
    n_saved = length(search_tree) - length(deleted)
    filter!(!in(deleted), sol.search_order)
    for del_id in deleted
        del_node = search_tree[del_id]
        del_node.path_cost = Inf32
        del_state = del_node.state
        # Iterate over parents
        parent_ref = del_node.parent
        del_node.parent = nothing
        while !isnothing(parent_ref)
            parent_id = parent_ref.id
            parent_act = parent_ref.action
            parent_ref = parent_ref.next
            # Skip parents that not in the interior of the new tree
            parent_id in deleted && continue
            parent_id in keys(search_tree) || continue
            parent_id in keys(queue) && continue
            # Update path cost and parent/child pointers
            parent = search_tree[parent_id]
            act_cost = get_cost(spec, domain, parent.state,
                                parent_act, del_state)
            path_cost = parent.path_cost + act_cost
            if path_cost < del_node.path_cost
                del_node.path_cost = path_cost
                del_node.parent =
                    LinkedNodeRef(parent_id, parent_act, del_node.parent)
                parent.child =
                    LinkedNodeRef(del_id, nothing, parent.child)
                push!(adopters, parent_id)
            else
                del_node.parent.next =
                    LinkedNodeRef(parent_id, parent_act, del_node.parent.next)
            end
        end
        # Delete node from search tree and frontier if not adopted
        in_queue = haskey(queue, del_id)
        in_queue || (sol.expanded -= 1)
        if isnothing(del_node.parent)
            delete!(search_tree, del_id)
            in_queue && delete!(queue, del_id)
        else # Place adopted node on the search frontier
            h_val::Float32 = in_queue ?
                queue[del_id][2] : compute(heuristic, domain, del_state, spec)
            f_val::Float32 = g_mult * del_node.path_cost + h_mult * h_val
            priority = (f_val, h_val, n_saved + n_adopted)
            queue[del_id] = priority
            n_adopted += 1
        end
    end
    # Deduplicate children of adoptive parents
    for parent_id in adopters
        parent = search_tree[parent_id]
        parent.child = unique(parent.child)
    end
    if verbose
        n_marked = length(deleted)
        n_deleted = length(deleted) - n_adopted
        n_adopters = length(adopters)
        stats_str = "marked = $n_marked, deleted = $n_deleted, " * 
            "adopted = $n_adopted, adopters = $n_adopters, saved = $n_saved"
        @logmsg cb.loglevel "Rerooting complete: " * stats_str
    end
    return sol
end

function (cb::LoggerCallback)(
    planner::ForwardPlanner,
    sol::PathSearchSolution, node_id::Union{UInt, Nothing}, priority
)
    node = isnothing(node_id) ? nothing : sol.search_tree[node_id]
    f, h, _ = priority
    g = isnothing(node) ? Inf32 : node.path_cost
    m, n = length(sol.search_tree), sol.expanded
    schedule = get(cb.options, :log_period_schedule,
                   [(10, 2), (100, 10), (1000, 100), (typemax(Int), 1000)])
    idx = findfirst(x -> n < x[1], schedule)
    log_period = isnothing(idx) ? 1000 : schedule[idx][2]
    if n <= 1 && get(cb.options, :log_header, true)
        @logmsg cb.loglevel "Starting forward search..."
        max_nodes, max_time = planner.max_nodes, planner.max_time
        @logmsg cb.loglevel "max_nodes = $max_nodes, max_time = $max_time"
        search_noise = planner.search_noise
        if !isnothing(search_noise)
            @logmsg cb.loglevel "search_noise = $search_noise"
        end
    end
    if n % log_period == 0 || sol.status != :in_progress
        @logmsg cb.loglevel "f = $f, g = $g, h = $h, $m evaluated, $n expanded"
    end
    if sol.status != :in_progress && get(cb.options, :log_solution, true)
        k = length(sol.plan)
        @logmsg cb.loglevel "Search terminated with status: $(sol.status)"
        if sol.status != :failure
            sol_str = sol.status == :success ? "Solution" : "Partial solution"
            init_node = sol.search_tree[hash(sol.trajectory[1])]
            init_cost = init_node.path_cost
            c = g - init_cost
            stats_str = iszero(init_cost) ?
                "$k actions, $c cost, $m evaluated, $n expanded" :
                "$k actions, $c cost ($g total), $m evaluated, $n expanded"
            @logmsg cb.loglevel "$sol_str: $stats_str"
        end
    end
    return nothing
end