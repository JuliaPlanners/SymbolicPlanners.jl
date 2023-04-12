export BreadthFirstPlanner

"""
    BreadthFirstPlanner(;
        max_nodes::Int = typemax(Int),
        max_time::Float64 = Inf,
        save_search::Bool = false,
        save_search_order::Bool = save_search
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
end

@auto_hash BreadthFirstPlanner
@auto_equals BreadthFirstPlanner

function Base.copy(p::BreadthFirstPlanner)
    return BreadthFirstPlanner(p.max_nodes, p.max_time,
                               p.save_search, p.save_search_order)
end

function solve(planner::BreadthFirstPlanner,
               domain::Domain, state::State, spec::Specification)
    @unpack save_search = planner
    # Simplify goal specification
    spec = simplify_goal(spec, domain, state)
    # Initialize backpointers and queue
    node_id = hash(state)
    search_tree = Dict(node_id => PathNode(node_id, state, 0.0))
    queue = [node_id]
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
        if is_goal(spec, domain, node.state)
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
        else # Reconstruct plan and return solution
            sol.plan, sol.trajectory = reconstruct(node_id, search_tree)
            return sol
        end
    end
    sol.status = :failure
    return sol
end

function expand!(planner::BreadthFirstPlanner, node::PathNode,
                 search_tree::Dict{UInt,<:PathNode}, queue::Vector{UInt},
                 domain::Domain, spec::Specification)
    state = node.state
    # Iterate over available actions
    for act in available(domain, state)
        # Execute actions on state
        next_state = execute(domain, state, act, check=false)
        next_id = hash(next_state)
        # Skip if state has already been encountered
        if haskey(search_tree, next_id) continue end
        # Check if next state satisfies trajectory constraints
        if is_violated(spec, domain, state) continue end
        # Update backpointer and add next state to queue
        path_cost = node.path_cost + 1
        search_tree[next_id] =
            PathNode(next_id, next_state, path_cost, node.id, act)
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
