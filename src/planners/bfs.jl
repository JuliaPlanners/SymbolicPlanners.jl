export BreadthFirstPlanner

"""
    BreadthFirstPlanner(;
        max_nodes::Int = typemax(Int),
        max_time::Float64 = Inf,
        save_search::Bool = false,
        save_search_order::Bool = save_search,
        verbose::Bool = false,
        callback = verbose ? LoggerCallback() : nothing
    )

Breadth-first search planner. Nodes are expanded in order of increasing distance
from the initial state (skipping previously visited nodes).

Returns a [`PathSearchSolution`](@ref) or [`NullSolution`](@ref), similar to
[`ForwardPlanner`](@ref).

# Arguments

$(FIELDS)
"""
@kwdef mutable struct BreadthFirstPlanner <: Planner
    "Maximum number of search nodes before termination."
    max_nodes::Int = typemax(Int)
    "Maximum time in seconds before planner times out."
    max_time::Float64 = Inf
    "Flag to save the search tree and frontier in the returned solution."
    save_search::Bool = false
    "Flag to save the node expansion order in the returned solution."
    save_search_order::Bool = save_search
    "Flag to print debug information during search."
    verbose::Bool = false
    "Callback function for logging, etc."
    callback::Union{Nothing, Function} = verbose ? LoggerCallback() : nothing
end

@auto_hash BreadthFirstPlanner
@auto_equals BreadthFirstPlanner

function Base.copy(p::BreadthFirstPlanner)
    return BreadthFirstPlanner(p.max_nodes, p.max_time,
                               p.save_search, p.save_search_order,
                               p.verbose, p.callback)
end

function solve(planner::BreadthFirstPlanner,
               domain::Domain, state::State, spec::Specification)
    @unpack save_search = planner
    # Simplify goal specification
    spec = simplify_goal(spec, domain, state)
    # Initialize backpointers and queue
    node_id = hash(state)
    node = PathNode(node_id, state, 0.0, LinkedNodeRef(node_id))
    search_tree = Dict(node_id => node)
    queue = [node_id]
    search_order = UInt[]
    sol = PathSearchSolution(:in_progress, Term[], Vector{typeof(state)}(),
                             0, search_tree, queue, search_order)
    # Check if initial state satisfies trajectory constraints
    if is_violated(spec, domain, state)
        sol.status = :failure
    else # Run the search
        sol = search!(sol, planner, domain, spec)
    end
    # Return solution
    if save_search
        return sol
    elseif sol.status == :failure
        return NullSolution(sol.status)
    else
        return PathSearchSolution(sol.status, sol.plan, sol.trajectory)
    end
end

function search!(sol::PathSearchSolution, planner::BreadthFirstPlanner,
                 domain::Domain, spec::Specification)
    start_time = time()
    sol.expanded = 0
    queue, search_tree = sol.search_frontier, sol.search_tree
    while length(queue) > 0
        # Look-up first state on queue
        node_id = first(queue)
        node = search_tree[node_id]
        # Check search termination criteria
        if is_goal(spec, domain, node.state, node.parent.action)
            sol.status = :success # Goal reached
        elseif sol.expanded >= planner.max_nodes
            sol.status = :max_nodes # Node budget reached
        elseif time() - start_time >= planner.max_time
            sol.status = :max_time # Time budget reached
        end
        if sol.status == :in_progress # Expand current node
            popfirst!(queue)
            expand!(planner, node, search_tree, queue, domain, spec)
            sol.expanded += 1
            if planner.save_search && planner.save_search_order
                push!(sol.search_order, node_id)
            end
            if !isnothing(planner.callback)
                planner.callback(planner, sol, node_id, sol.expanded)
            end
        else # Reconstruct plan and return solution
            sol.plan, sol.trajectory = reconstruct(node_id, search_tree)
            if !isnothing(planner.callback)
                planner.callback(planner, sol, node_id, sol.expanded)
            end
            return sol
        end
    end
    sol.status = :failure
    return sol
end

function expand!(
    planner::BreadthFirstPlanner, node::PathNode{S},
    search_tree::Dict{UInt,PathNode{S}}, queue::Vector{UInt},
    domain::Domain, spec::Specification
) where {S <: State}
    state = node.state
    # Iterate over available actions
    for act in available(domain, state)
        # Execute actions on state
        next_state = transition(domain, state, act, check=false)
        next_id = hash(next_state)
        # Check if action goal is reached
        if has_action_goal(spec) && is_goal(spec, domain, next_state, act)
            next_id = hash((next_state, act))
        end
        # Skip if state has already been encountered
        if haskey(search_tree, next_id) continue end
        # Check if next state satisfies trajectory constraints
        if is_violated(spec, domain, next_state) continue end
        # Update backpointer and add next state to queue
        path_cost = node.path_cost + 1
        search_tree[next_id] = PathNode(next_id, next_state, path_cost,
                                        LinkedNodeRef(node.id, act))
        push!(queue, next_id)
    end
end

function refine!(
    sol::PathSearchSolution{S, T}, planner::BreadthFirstPlanner,
    domain::Domain, state::State, spec::Specification
) where {S, T <: Vector}
    sol.status == :success && return sol
    sol.status = :in_progress
    spec = simplify_goal(spec, domain, state)
    return search!(sol, planner, domain, spec)
end

function (cb::LoggerCallback)(
    planner::BreadthFirstPlanner,
    sol::PathSearchSolution, node_id::UInt, args...
)
    g = sol.search_tree[node_id].path_cost
    m, n = length(sol.search_tree), sol.expanded
    schedule = get(cb.options, :log_period_schedule,
                   [(10, 2), (100, 10), (1000, 100), (typemax(Int), 1000)])
    idx = findfirst(x -> n < x[1], schedule)
    log_period = isnothing(idx) ? 1000 : schedule[idx][2]
    if n == 1 && get(cb.options, :log_header, true)
        @logmsg cb.loglevel "Starting breadth-first search..."
        max_nodes, max_time = planner.max_nodes, planner.max_time
        @logmsg cb.loglevel "max_nodes = $max_nodes, max_time = $max_time" 
    end
    if n % log_period == 0 || sol.status != :in_progress
        @logmsg cb.loglevel "g = $g, $m evaluated, $n expanded"
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
