export BiDirectionalPlanner, BiGreedyPlanner, BiAStarPlanner

"BiDirectionalPlanner best-first search planner."
mutable struct BiDirectionalPlanner{F,B} <: Planner
    forward::F
    backward::B
end

BiDirectionalPlanner(;kwargs...) = BiDirectionalPlanner(
    	ForwardPlanner(; kwargs...),
    	BackwardPlanner(; kwargs...)
    )

"Bidirectional  Greedy best-first search, with cycle checking."
BiGreedyPlanner(heuristic::Heuristic; kwargs...) =
    BiDirectionalPlanner(
    	ForwardPlanner(;heuristic=heuristic, g_mult=0, kwargs...),
    	BackwardPlanner(;heuristic=heuristic, g_mult=0, kwargs...)
    )


"Bidirectional A* search."
BiAStarPlanner(heuristic::Heuristic; kwargs...) =
    BiDirectionalPlanner(
    	ForwardPlanner(;heuristic=heuristic, kwargs...),
    	BackwardPlanner(;heuristic=heuristic, kwargs...)
    )

function solve(planner::BiDirectionalPlanner,
               domain::Domain, state::State, spec::Specification)

    # Simplify goal specification
    forward_spec = simplify_goal(spec, domain, state)
    backward_spec = BackwardSearchGoal(spec, state)
    # Precompute heuristic information
    precompute!(planner.forward.heuristic, domain, state, forward_spec)
    precompute!(planner.backward.heuristic, domain, state, backward_spec)
    
	forward_search_tree, forward_queue = init_forward(planner.forward, domain, state, spec)
	backward_search_tree, backward_queue = init_backward(planner.backward, domain, state, spec)

    status, node_id, count = search!(planner, domain, 
    	forward_spec, forward_search_tree, forward_queue,
    	backward_spec, backward_search_tree, backward_queue)

    # Reconstruct plan and return solution
    if status != :failure
        forward_plan, forward_traj = reconstruct(node_id, forward_search_tree)
        backward_plan, backward_traj = reconstruct(node_id, backward_search_tree)
        reverse!(backward_plan); reverse!(backward_traj)
        plan = vcat(forward_plan, backward_plan)
        traj = vcat(forward_traj, backward_traj)
        if planner.forward.save_search || planner.backward.save_search
            return BiPathSearchSolution(status, plan, traj,
                                      count, forward_search_tree, forward_queue, 
                                      backward_search_tree, backward_queue)
        else
            return BiPathSearchSolution(status, plan, traj)
        end
    elseif planner.forward.save_search || planner.backward.save_search
        S = typeof(state)
        return BiPathSearchSolution(status, Term[], S[],
                                  count, search_tree, queue)
    else
        return NullSolution(status)
    end
end

function search!(planner::BiDirectionalPlanner, domain::Domain, 
                 forward_spec::Specification, forward_search_tree::Dict{UInt,<:PathNode}, forward_queue::PriorityQueue,
                 backward_spec::Specification, backward_search_tree::Dict{UInt,<:PathNode}, backward_queue::PriorityQueue)
	max_nodes = max(planner.forward.max_nodes)
	max_time = max(planner.forward.max_time)
    count = 2
    start_time = time()
    while length(forward_queue) > 0
    	# progress the forward part 
        # Get state with lowest estimated cost to goal
        node_id = dequeue!(forward_queue)
        node = forward_search_tree[node_id]
        if is_goal(forward_spec, domain, node.state) || node_id ∈ keys(backward_search_tree)
            return :success, node_id, count # Goal reached
        end
        expand!(planner.forward, node, forward_search_tree, forward_queue, domain, forward_spec)
        count += 1

    	# progress the backward part 
        # Get state with lowest estimated cost to goal
        node_id = dequeue!(backward_queue)
        node = backward_search_tree[node_id]
        if is_goal(backward_spec, domain, node.state) || node_id ∈ keys(forward_search_tree)
            return :success, node_id, count # Goal reached
        end
        expand!(planner.backward, node, backward_search_tree, backward_queue, domain, backward_spec)
        count += 1

        # check if we have not exceeded allowed time
        if count >= max_nodes
            return :max_nodes, node_id, count # Node budget reached
        elseif time() - start_time >= max_time
            return :max_time, node_id, count # Time budget reached
        end
    end
    return :failure, nothing, count
end

function init_backward(planner::BackwardPlanner,
               domain::Domain, state::State, spec::Specification)
    @unpack h_mult, heuristic, save_search = planner
    spec = BackwardSearchGoal(spec, state)
    state = goalstate(domain, PDDL.get_objtypes(state), get_goal_terms(spec))
    # Initialize search tree and priority queue
    node_id = hash(state)
    search_tree = Dict(node_id => PathNode(node_id, state, 0.0))
    est_cost::Float32 = h_mult * compute(heuristic, domain, state, spec)
    priority = (est_cost, est_cost, 0)
    queue = PriorityQueue(node_id => priority)
    return(search_tree, queue)
end

function init_forward(planner::ForwardPlanner,
               domain::Domain, state::State, spec::Specification)
	@unpack h_mult, heuristic, save_search = planner
    node_id = hash(state)
    search_tree = Dict(node_id => PathNode(node_id, state, 0.0))
    est_cost::Float32 = h_mult * compute(heuristic, domain, state, spec)
    priority = (est_cost, est_cost, 0)
    queue = PriorityQueue(node_id => priority)
    return(search_tree, queue)
end
