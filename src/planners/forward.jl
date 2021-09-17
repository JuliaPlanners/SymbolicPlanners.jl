export ForwardPlanner, BestFirstPlanner, UniformCostPlanner, GreedyPlanner
export AStarPlanner, WeightedAStarPlanner

"Forward best-first search planner."
@kwdef mutable struct ForwardPlanner <: Planner
    heuristic::Heuristic = GoalCountHeuristic()
    g_mult::Float64 = 1.0 # Path cost multiplier
    h_mult::Float64 = 1.0 # Heuristic multiplier
    max_nodes::Int = typemax(Int)
    save_search::Bool = false # Flag to save search info
end

"Best-first search planner (alias for `ForwardPlanner`)."
BestFirstPlanner(args...; kwargs...) =
    ForwardPlanner(args...; kwargs...)

"Uniform-cost search."
UniformCostPlanner(;kwargs...) =
    ForwardPlanner(;heuristic=NullHeuristic(), h_mult=0, kwargs...)

"Greedy best-first search, with cycle checking."
GreedyPlanner(heuristic::Heuristic; kwargs...) =
    ForwardPlanner(;heuristic=heuristic, g_mult=0, kwargs...)

"A* search."
AStarPlanner(heuristic::Heuristic; kwargs...) =
    ForwardPlanner(;heuristic=heuristic, kwargs...)

"Weighted A* search."
WeightedAStarPlanner(heuristic::Heuristic, h_mult::Real; kwargs...) =
    ForwardPlanner(;heuristic=heuristic, h_mult=h_mult, kwargs...)

function solve(planner::ForwardPlanner,
               domain::Domain, state::State, spec::Specification)
    @unpack h_mult, heuristic, save_search = planner
    # Precompute heuristic information
    precompute!(heuristic, domain, state, spec)
    # Initialize search tree and priority queue
    node_id = hash(state)
    search_tree = Dict(node_id => PathNode(node_id, state, 0.0))
    est_cost = h_mult * heuristic(domain, state, spec)
    priority = (est_cost, est_cost, 0)
    queue = PriorityQueue(node_id => priority)
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
        return NullSolution()
    end
end

function search!(planner::ForwardPlanner,
                 domain::Domain, spec::Specification,
                 search_tree::Dict{UInt,<:PathNode}, queue::PriorityQueue)
    count = 1
    while length(queue) > 0
        # Get state with lowest estimated cost to goal
        node_id = dequeue!(queue)
        node = search_tree[node_id]
        # Return status and current state if search terminates
        if is_goal(spec, domain, node.state)
            return :success, node_id, count # Goal reached
        elseif count >= planner.max_nodes
            return :max_nodes, node_id, count # Node budget reached
        end
        count += 1
        # Expand current node
        expand!(planner, node, search_tree, queue, domain, spec)
    end
    return :failure, nothing, count
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
                         PathNode(next_id, next_state, Inf))
        cost_diff = next_node.path_cost - path_cost
        if cost_diff > 0
            next_node.parent_id = node.id
            next_node.parent_action = act
            next_node.path_cost = path_cost
            # Update estimated cost from next state to goal
            if !(next_id in keys(queue))
                g_val::Float64 = g_mult * path_cost
                h_val::Float64 = h_mult * heuristic(domain, next_state, spec)
                f_val = g_val + h_val
                priority = (f_val, h_val, length(search_tree))
                enqueue!(queue, next_id, priority)
            else
                f_val, h_val, n_nodes = queue[next_id]
                queue[next_id] = (f_val - cost_diff, h_val, n_nodes)
                # queue[next_id] -= cost_diff
            end
        end
    end
end
