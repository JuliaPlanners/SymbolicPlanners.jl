export BreadthFirstPlanner

"Uninformed breadth-first search planner."
@kwdef mutable struct BreadthFirstPlanner <: Planner
    max_nodes::Int = typemax(Int)
    save_search::Bool = false # Flag to save search info
end

function solve(planner::BreadthFirstPlanner,
               domain::Domain, state::State, spec::Specification)
    @unpack max_nodes, save_search = planner
    # Initialize backpointers and queue
    node_id = hash(state)
    search_tree = SearchTree(node_id => SearchNode(node_id, state, 0))
    queue = [node_id]
    # Run the search
    status, node_id = search!(planner, domain, spec, search_tree, queue)
    # Reconstruct plan and return solution
    if status != :failure
        plan, traj = reconstruct(node_id, search_tree)
        if save_search
            return OrderedSearchSolution(status, plan, traj, search_tree, queue)
        else
            return OrderedSearchSolution(status, plan, traj)
        end
    elseif save_search
        return OrderedSearchSolution(status, [], [], search_tree, queue)
    else
        return NullSolution()
    end
end

function search!(planner::BreadthFirstPlanner,
                 domain::Domain, spec::Specification,
                 search_tree::SearchTree, queue::Vector{UInt})
    count = 1
    while length(queue) > 0
        # Pop state off the queue
        node_id = popfirst!(queue)
        node = search_tree[node_id]
        # Return if max nodes are reached or goals are satisfied
        if is_goal(spec, domain, node.state)
            return :success, node_id
        elseif count >= planner.max_nodes
            return :max_nodes, node_id
        end
        count += 1
        # Expand current node
        expand!(planner, node, search_tree, queue, domain, spec)
    end
    return :failure, nothing
end

function expand!(planner::BreadthFirstPlanner, node::SearchNode,
                 search_tree::SearchTree, queue::Vector{UInt},
                 domain::Domain, spec::Specification)
    state = node.state
    # Iterate over available actions
    actions = available(state, domain)
    for act in actions
        # Execute actions on state
        next_state = transition(domain, state, act; check=false)
        next_id = hash(next_state)
        # Skip if state has already been encountered
        if haskey(search_tree, next_id) continue end
        # Check if next state satisfies trajectory constraints
        if is_violated(spec, domain, state) continue end
        # Update backpointer and add next state to queue
        search_tree[next_id] = SearchNode(next_id, next_state, 0, node.id, act)
        push!(queue, next_id)
    end
end
