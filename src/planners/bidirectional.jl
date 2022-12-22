export BidirectionalPlanner, BiGreedyPlanner, BiAStarPlanner

"BidirectionalPlanner best-first search planner."
@kwdef mutable struct BidirectionalPlanner <: Planner
    forward::ForwardPlanner = ForwardPlanner()
    backward::BackwardPlanner = BackwardPlanner()
    max_nodes::Int = typemax(Int) # Max search nodes before termination
    max_time::Float64 = Inf # Max time in seconds before timeout
    save_search::Bool = false # Flag to save search info
end    

"Bidirectional  Greedy best-first search, with cycle checking."
BiGreedyPlanner(f_heuristic::Heuristic, b_heuristic::Heuristic;  kwargs...) = 
    BidirectionalPlanner(
    	ForwardPlanner(;heuristic=f_heuristic, g_mult=0, kwargs...),
    	BackwardPlanner(;heuristic=b_heuristic, g_mult=0, kwargs...),
        kwargs...
    )


BidirectionalPlanner(f_heuristic::Heuristic, b_heuristic::Heuristic; max_nodes = typemax(Int64), max_time = Inf, save_search = false) = 
    BidirectionalPlanner(
        ForwardPlanner(;heuristic=f_heuristic, max_nodes, max_time, save_search),
        BackwardPlanner(;heuristic=b_heuristic, max_nodes, max_time, save_search),
        max_nodes, 
        max_time, 
        save_search
    )


"Bidirectional A* search."
BiAStarPlanner(f_heuristic::Heuristic, b_heuristic::Heuristic; kwargs...) =
    BidirectionalPlanner(
    	ForwardPlanner(;heuristic=f_heuristic, kwargs...),
    	BackwardPlanner(;heuristic=b_heuristic, kwargs...),
        kwargs...
    )

function solve(planner::BidirectionalPlanner,
               domain::Domain, state::State, spec::Specification)

    # Simplify goal specification
    forward_spec = simplify_goal(spec, domain, state)
    backward_spec = BackwardSearchGoal(spec, state)
    # Precompute heuristic information
    precompute!(planner.forward.heuristic, domain, state, forward_spec)
    precompute!(planner.backward.heuristic, domain, state, backward_spec)
    
	f_search_tree, f_queue = init_forward(planner.forward, domain, state, spec)
	b_search_tree, b_queue = init_backward(planner.backward, domain, state, spec)

    status, node_id, count = search!(planner, domain, 
    	forward_spec, f_search_tree, f_queue,
    	backward_spec, b_search_tree, b_queue)

    # Reconstruct plan and return solution
    if status != :failure
        f_plan, f_trajectory = reconstruct(node_id, f_search_tree)
        b_plan, b_trajectory = reconstruct(node_id, b_search_tree)
        plan = vcat(f_plan, reverse(b_plan))
        traj = simulate(StateRecorder(), domain, state, plan)

        if planner.save_search
            return BiPathSearchSolution(status, plan, traj, count, 
                            f_search_tree, f_queue, length(f_search_tree), f_trajectory,
                            b_search_tree, b_queue, length(b_search_tree), b_trajectory)
        else
            return BiPathSearchSolution(status, plan, traj)
        end
    elseif planner.save_search
        S = typeof(state)
        return BiPathSearchSolution(status, Term[], S[],
                                  count, search_tree, queue)
    else
        return NullSolution(status)
    end
end

function search!(planner::BidirectionalPlanner, domain::Domain, 
                 forward_spec::Specification, f_search_tree::Dict{UInt,<:PathNode}, f_queue::PriorityQueue,
                 backward_spec::Specification, b_search_tree::Dict{UInt,<:PathNode}, b_queue::PriorityQueue)
	max_nodes = planner.max_nodes
	max_time = planner.max_time
    count = 2
    start_time = time()
    while length(f_queue) > 0
    	# progress the forward part 
        # Get state with lowest estimated cost to goal
        node_id = dequeue!(f_queue)
        node = f_search_tree[node_id]
        if is_goal(forward_spec, domain, node.state) || any(issubset(b_search_tree[first(fnode)].state, node.state) for fnode in b_queue)
            return :success, node_id, count # Goal reached
        end

        expand!(planner.forward, node, f_search_tree, f_queue, domain, forward_spec)
        count += 1

    	# progress the backward part 
        # Get state with lowest estimated cost to goal
        node_id = dequeue!(b_queue)
        node = b_search_tree[node_id]
        if is_goal(backward_spec, domain, node.state) || any(issubset(node.state, f_search_tree[first(fnode)].state) for fnode in f_queue)
            return :success, node_id, count # Goal reached
        end
        expand!(planner.backward, node, b_search_tree, b_queue, domain, backward_spec)
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