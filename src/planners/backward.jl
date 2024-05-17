export BackwardPlanner, BackwardGreedyPlanner, BackwardAStarPlanner
export ProbBackwardPlanner, ProbBackwardAStarPlanner

"""
    BackwardPlanner(;
        heuristic::Heuristic = GoalCountHeuristic(:backward),
        search_noise::Union{Nothing,Float64} = nothing,
        g_mult::Float32 = 1.0f0,
        h_mult::Float32 = 1.0f0,
        max_nodes::Int = typemax(Int),
        max_time::Float64 = Inf,
        fail_fast::Bool = false,
        save_search::Bool = false,
        save_search_order::Bool = save_search,
        verbose::Bool = false,
        callback = verbose ? LoggerCallback() : nothing
    )

Heuristic-guided backward (i.e. regression) search planner. Instead of searching
forwards, searches backwards from the goal, which is treated as a *set* of 
states which satisfy the goal predicates (equivalently, a *partial* state, 
because only some predicates and fluents may be specified). Each expanded node
also corresponds to a partial state. [1]

As with [`ForwardPlanner`](@ref), each node ``n`` is expanded in order of
increasing priority ``f(n)``, defined as:

```math
f(n) = g_\\text{mult} \\cdot g(n) + h_\\text{mult} \\cdot h(n)
```

However ``g(n)`` is instead defined as the path cost from the goal to the
current node ``n``, and ``h(n)`` is a heuristic estimate of the distance
from the initial state. As such, only certain heuristics, such as
[`GoalCountHeuristic`](@ref) and  [`HSPRHeuristic`](@ref) can be used with
backward search.

Returns a [`PathSearchSolution`](@ref) or [`NullSolution`](@ref), similar to
[`ForwardPlanner`](@ref).

This planner does not currently support domains with non-Boolean fluents.

[1] B. Bonet and H. Geffner, "Planning as Heuristic Search," Artificial
Intelligence, vol. 129, no. 1, pp. 5–33, Jun. 2001,
<https://doi.org/10.1016/S0004-3702(01)00108-4>.

# Arguments

$(FIELDS)
"""
@kwdef mutable struct BackwardPlanner{T <: Union{Nothing, Float64}} <: Planner
    "Search heuristic that estimates cost of a state to the goal."
    heuristic::Heuristic = GoalCountHeuristic(:backward)
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
    "Flag to save the search tree and frontier in the returned solution."
    save_search::Bool = false
    "Flag to save the node expansion order in the returned solution."
    save_search_order::Bool = save_search
    "Flag to print debug information during search."
    verbose::Bool = false
    "Callback function for logging, etc."
    callback::Union{Nothing, Function} = verbose ? LoggerCallback() : nothing
end

@auto_hash BackwardPlanner
@auto_equals BackwardPlanner

BackwardPlanner(heuristic::Heuristic, search_noise::T, args...) where {T} =
    BackwardPlanner{T}(heuristic, search_noise, args...)

"""
$(SIGNATURES)

Backward greedy search, with cycle checking.
"""
BackwardGreedyPlanner(heuristic::Heuristic; kwargs...) =
    BackwardPlanner(;heuristic=heuristic, g_mult=0, kwargs...)

"""
$(SIGNATURES)

Backward A* search.
"""
BackwardAStarPlanner(heuristic::Heuristic; kwargs...) =
    BackwardPlanner(;heuristic=heuristic, kwargs...)

"""
    ProbBackwardPlanner(;
        search_noise::Float64 = 1.0,
        kwargs...
    )

A probabilistic variant of backward search, with the same node expansion
rule as [`ProbForwardPlanner`](@ref).

An alias for `BackwardPlanner{Float64}`. See [`BackwardPlanner`](@ref) for
other arguments.
"""
const ProbBackwardPlanner = BackwardPlanner{Float64}

ProbBackwardPlanner(;search_noise=1.0, kwargs...) = 
    BackwardPlanner(;search_noise=search_noise, kwargs...)

"""
$(SIGNATURES)

A probabilistic variant of backward A* search.
"""
ProbBackwardAStarPlanner(heuristic::Heuristic; search_noise=1.0, kwargs...) =
    BackwardPlanner(;heuristic=heuristic, search_noise=search_noise, kwargs...)

function Base.copy(p::BackwardPlanner)
    return BackwardPlanner(p.heuristic, p.search_noise,
                           p.g_mult, p.h_mult, p.max_nodes, p.max_time,
                           p.fail_fast, p.save_search, p.save_search_order,
                           p.verbose, p.callback)
end

function solve(planner::BackwardPlanner,
               domain::Domain, state::State, spec::Specification)
    @unpack h_mult, heuristic, save_search = planner
    # Precompute heuristic information
    precompute!(heuristic, domain, state, spec)
    # Simplify goal specification
    spec = simplify_goal(spec, domain, state)
    # Convert to backward search goal specification
    spec = BackwardSearchGoal(spec, domain, state)
    state = goalstate(domain, PDDL.get_objtypes(state), get_goal_terms(spec))
    # Construct initial search node
    node_id = hash(state)
    node = PathNode(node_id, state, 0.0)
    # Initialize search tree and priority queue
    search_tree = Dict(node_id => node)
    h_val::Float32 = compute(heuristic, domain, state, spec)
    priority = (h_mult * h_val, h_val, 0)
    queue = PriorityQueue(node_id => priority)
    search_order = UInt[]
    sol = PathSearchSolution(:in_progress, Term[], Vector{typeof(state)}(),
                             0, search_tree, queue, search_order)
    # Run the search
    sol = search!(sol, planner, planner.heuristic, domain, spec)
    # Return solution
    if save_search
        return sol
    elseif sol.status == :failure
        return NullSolution(sol.status)
    else
        return PathSearchSolution(sol.status, sol.plan, sol.trajectory)
    end
end

function search!(sol::PathSearchSolution,
                 planner::BackwardPlanner, heuristic::Heuristic,
                 domain::Domain, spec::BackwardSearchGoal)
    @unpack search_noise = planner
    start_time = time()
    sol.expanded = 0
    queue, search_tree = sol.search_frontier, sol.search_tree
    while length(queue) > 0
        # Get state with lowest estimated cost to goal
        node_id, priority = isnothing(search_noise) ?
            peek(queue) : prob_peek(queue, search_noise)
        node = search_tree[node_id]
        # Check search termination criteria
        if is_goal(spec, domain, node.state)
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
            isnothing(search_noise) ? dequeue!(queue) : dequeue!(queue, node_id) 
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
            reverse!(sol.plan)
            reverse!(sol.trajectory)
            if !isnothing(planner.callback)
                planner.callback(planner, sol, node_id, priority)
            end
            return sol
        end
    end
    sol.status = :failure
    return sol
end

function expand!(
    planner::BackwardPlanner, heuristic::Heuristic, node::PathNode{S},
    search_tree::Dict{UInt, PathNode{S}}, queue::PriorityQueue,
    domain::Domain, spec::BackwardSearchGoal
) where {S <: State}
    @unpack g_mult, h_mult = planner
    state = node.state
    # Iterate over relevant actions, filtered by heuristic
    for act in filter_relevant(heuristic, domain, state, spec)
        # Regress (reverse-execute) the action
        next_state = regress(domain, state, act; check=false)
        # Add constraints to regression state
        add_constraints!(spec, domain, state)
        next_id = hash(next_state)
        # Compute path cost
        act_cost = get_cost(spec, domain, state, act, next_state)
        path_cost = node.path_cost + act_cost
        # Construct or retrieve child node
        next_node = get!(search_tree, next_id) do
            PathNode{S}(next_id, next_state, Inf32)
        end
        cost_diff = next_node.path_cost - path_cost
        if cost_diff > 0 # Update path costs if new path is shorter
            next_node.path_cost = path_cost
            # Update parent pointer
            next_node.parent = LinkedNodeRef(node.id, act)
            # Update estimated cost from next state to start
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
    sol::PathSearchSolution{S, T}, planner::BackwardPlanner,
    domain::Domain, state::State, spec::Specification
) where {S, T <: PriorityQueue}
    sol.status == :success && return sol
    sol.status = :in_progress
    spec = simplify_goal(spec, domain, state)
    spec = BackwardSearchGoal(spec, domain, state)
    ensure_precomputed!(planner.heuristic, domain, state, spec)
    return search!(sol, planner, planner.heuristic, domain, spec)
end

function (cb::LoggerCallback)(
    planner::BackwardPlanner,
    sol::PathSearchSolution, node_id::UInt, priority
)
    f, h, _ = priority
    g = sol.search_tree[node_id].path_cost
    m, n = length(sol.search_tree), sol.expanded
    schedule = get(cb.options, :log_period_schedule,
                   [(10, 2), (100, 10), (1000, 100), (typemax(Int), 1000)])
    idx = findfirst(x -> n < x[1], schedule)
    log_period = isnothing(idx) ? 1000 : schedule[idx][2]
    if n == 1 && get(cb.options, :log_header, true)
        @logmsg cb.loglevel "Starting backward search..."
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
            sol_txt = sol.status == :success ? "Solution" : "Partial solution"
            @logmsg cb.loglevel "$sol_txt: $k actions, $g cost, $m evaluated, $n expanded"
        end
    end
    return nothing
end
