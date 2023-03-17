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
        save_search::Bool = false,
        save_search_order::Bool = false
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
"""
@kwdef mutable struct ForwardPlanner{T <: Union{Nothing, Float64}} <: Planner
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
    "Flag to save the search tree and frontier in the returned solution."
    save_search::Bool = false
    "Flag to save the node expansion order in the returned solution."
    save_search_order::Bool = false
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
    return ForwardPlanner(p.heuristic, p.search_noise, 
                          p.g_mult, p.h_mult, p.max_nodes, p.max_time,
                          p.save_search, p.save_search_order)
end

function solve(planner::ForwardPlanner,
               domain::Domain, state::State, spec::Specification)
    @unpack h_mult, heuristic, save_search = planner
    # Simplify goal specification
    spec = simplify_goal(spec, domain, state)
    # Precompute heuristic information
    precompute!(heuristic, domain, state, spec)
    # Initialize search tree and priority queue
    node_id = hash(state)
    search_tree = Dict(node_id => PathNode(node_id, state, 0.0))
    est_cost::Float32 = h_mult * compute(heuristic, domain, state, spec)
    priority = (est_cost, est_cost, 0)
    queue = PriorityQueue(node_id => priority)
    search_order = UInt[]
    sol = PathSearchSolution(:in_progress, Term[], Vector{typeof(state)}(),
                             0, search_tree, queue, search_order)
    # Run the search
    sol = search!(sol, planner, domain, spec)
    # Return solution
    if save_search
        return sol
    elseif sol.status == :failure
        return NullSolution(sol.status)
    else
        return PathSearchSolution(sol.status, sol.plan, sol.trajectory)
    end
end

function search!(sol::PathSearchSolution, planner::ForwardPlanner,
                 domain::Domain, spec::Specification)
    @unpack search_noise = planner
    start_time = time()
    sol.expanded = 0
    queue, search_tree = sol.search_frontier, sol.search_tree
    while length(queue) > 0
        # Get state with lowest estimated cost to goal
        node_id, _ = isnothing(search_noise) ?
            peek(queue) : prob_peek(queue, search_noise)
        node = search_tree[node_id]
        # Check search termination criteria
        if is_goal(spec, domain, node.state)
            sol.status = :success # Goal reached
        elseif sol.expanded >= planner.max_nodes
            sol.status = :max_nodes # Node budget reached
        elseif time() - start_time >= planner.max_time
            sol.status = :max_time # Time budget reached
        end
        if sol.status == :in_progress
            # Dequeue current node
            isnothing(search_noise) ? dequeue!(queue) : delete!(queue, node_id) 
            # Expand current node
            expand!(planner, node, search_tree, queue, domain, spec)
            sol.expanded += 1
            if planner.save_search && planner.save_search_order
                push!(sol.search_order, node_id)
            end
        else # Reconstruct plan and return solution
            sol.plan, sol.trajectory = reconstruct(node_id, search_tree)
            return sol
        end
    end
    sol.status = :failure
    return sol
end

function expand!(planner::ForwardPlanner, node::PathNode,
                 search_tree::Dict{UInt,<:PathNode}, queue::PriorityQueue,
                 domain::Domain, spec::Specification)
    @unpack g_mult, h_mult, heuristic = planner
    state = node.state
    # Iterate over available actions
    for act in available(domain, state)
        # Execute action and trigger all post-action events
        next_state = execute(domain, state, act; check=false)
        next_id = hash(next_state)
        # Check if next state satisfies trajectory constraints
        if is_violated(spec, domain, state) continue end
        # Compute path cost
        act_cost = get_cost(spec, domain, state, act, next_state)
        path_cost = node.path_cost + act_cost
        # Update path costs if new path is shorter
        next_node = get!(search_tree, next_id,
                         PathNode(next_id, next_state, Inf32))
        cost_diff = next_node.path_cost - path_cost
        if cost_diff > 0
            next_node.parent_id = node.id
            next_node.parent_action = act
            next_node.path_cost = path_cost
            # Update estimated cost from next state to goal
            if !(next_id in keys(queue))
                h_val::Float32 = compute(heuristic, domain, next_state, spec)
                f_val::Float32 = g_mult * path_cost + h_mult * h_val
                priority = (f_val, h_val, length(search_tree))
                enqueue!(queue, next_id, priority)
            else
                f_val, h_val, n_nodes = queue[next_id]
                queue[next_id] = (f_val - cost_diff, h_val, n_nodes)
            end
        end
    end
end

function refine!(
    sol::PathSearchSolution{S, T}, planner::ForwardPlanner,
    domain::Domain, state::State, spec::Specification
) where {S, T <: PriorityQueue}
    # TODO : re-root search tree at new state?
    sol.status == :success && return sol
    sol.status = :in_progress
    spec = simplify_goal(spec, domain, state)
    ensure_precomputed!(planner.heuristic, domain, state, spec)
    return search!(sol, planner, domain, spec)
end
