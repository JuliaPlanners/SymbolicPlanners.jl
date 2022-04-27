export BreadthFirstPlanner

"Uninformed breadth-first search planner."
@kwdef mutable struct BreadthFirstPlanner <: Planner
    max_nodes::Int = typemax(Int)
    max_time::Float64 = Inf # Max time in seconds before timeout
    save_search::Bool = false # Flag to save search info
end

function solve(planner::BreadthFirstPlanner,
               domain::Domain, state::State, spec::Specification)
    @unpack max_nodes, save_search = planner
    # Initialize backpointers and queue
    node_id = hash(state)
    search_tree = Dict(node_id => PathNode(node_id, state, 0.0))
    queue = [node_id]
    # Run the search
    status, node_id, count = search!(planner, domain, spec, search_tree, queue)
    # Reconstruct plan and return solution
    if status != :failure
        plan, traj = reconstruct(node_id, search_tree)
        if save_search
            return PathSearchSolution(status, plan, traj,
                                      count, search_tree, queue)
        else
            return PathSearchSolution(status, plan, traj)
        end
    elseif save_search
        S = typeof(state)
        return PathSearchSolution(status, Term[], S[],
                                  count, search_tree, queue)
    else
        return NullSolution(status)
    end
end

function search!(planner::BreadthFirstPlanner,
                 domain::Domain, spec::Specification,
                 search_tree::Dict{UInt,<:PathNode}, queue::Vector{UInt})
    count = 1
    start_time = time()
    while length(queue) > 0
        # Pop state off the queue
        node_id = popfirst!(queue)
        node = search_tree[node_id]
        # Return status and current state if search terminates
        if is_goal(spec, domain, node.state)
            return :success, node_id, count # Goal reached
        elseif count >= planner.max_nodes
            return :max_nodes, node_id, count # Node budget reached
        elseif time() - start_time >= planner.max_time
            return :max_time, node_id, count # Time budget reached
        end
        count += 1
        # Expand current node
        expand!(planner, node, search_tree, queue, domain, spec)
    end
    return :failure, nothing, count
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
