export BackwardPlanner, BackwardGreedyPlanner, BackwardAStarPlanner

"Heuristic-guided backward best-first search."
@kwdef mutable struct BackwardPlanner <: Planner
    heuristic::Heuristic = GoalCountHeuristic(:backward)
    g_mult::Float64 = 1.0 # Path cost multiplier
    h_mult::Float64 = 1.0 # Heuristic multiplier
    max_nodes::Int = typemax(Int)
    save_search::Bool = false # Flag to save search info
end

"Backward A* search for a plan."
function call(planner::BackwardPlanner,
              domain::Domain, state::State, goal_spec::GoalSpec)
    @unpack goals, metric, constraints = goal_spec
    @unpack max_nodes, g_mult, h_mult, heuristic, save_search = planner
    # Perform any precomputation required by the heuristic
    heuristic = precompute!(heuristic, domain, state, goal_spec)
    # Construct references to start and goal states
    start = state
    state = State(goal_spec.goals, PDDL.get_types(start))
    # Construct diff of constraints
    constraints = isempty(constraints) ? nothing : precond_diff(constraints)
    # Initialize search tree and priority queue
    state_hash = hash(state)
    search_tree = SearchTree(state_hash => SearchNode(state, 0))
    est_cost = heuristic(domain, state, goal_spec)
    queue = PriorityQueue{UInt,Float64}(state_hash => est_cost)
    count = 1
    while length(queue) > 0
        # Get state with lowest estimated cost to goal
        state_hash = dequeue!(queue)
        curr_node = search_tree[state_hash]
        state = curr_node.state
        # Return if initial state is implied or search budget is reached
        if issubset(state, start)
            plan, traj = reconstruct(state_hash, search_tree)
            reverse!(plan); reverse!(traj)
            return save_search ?
                SearchSolution(:success, plan, traj, search_tree, queue) :
                SearchSolution(:success, plan, traj)
        elseif count >= max_nodes
            return save_search ?
                SearchSolution(:max_nodes, [], [], search_tree, queue) :
                NullSolution()
        end
        count += 1
        # Get list of relevant actions
        actions = relevant(state, domain)
        # Iterate over actions
        for act in actions
            # Regress (reverse-execute) the action
            prev_state = regress(act, state, domain; check=false)
            # Add constraints to regression state
            if (constraints != nothing) update!(prev_state, constraints) end
            prev_hash = hash(prev_state)
            # Compute path cost
            act_cost = metric == nothing ? 1 :
                state[domain, metric] - prev_state[domain, metric]
            path_cost = curr_node.path_cost + act_cost
            # Update path costs if new path is shorter
            prev_node = get!(search_tree, prev_hash, SearchNode(prev_state, Inf))
            cost_diff = prev_node.path_cost - path_cost
            if cost_diff > 0
                prev_node.parent_hash = state_hash
                prev_node.parent_action = act
                prev_node.path_cost = path_cost
                # Update estimated cost from prev state to start
                if !(prev_hash in keys(queue))
                    g_val = g_mult * path_cost
                    h_val = h_mult * heuristic(domain, prev_state, goal_spec)
                    enqueue!(queue, prev_hash, g_val + h_val)
                else
                    queue[prev_hash] -= cost_diff
                end
            end
        end
    end
    return save_search ?
        SearchSolution(:failure, [], [], search_tree, queue) : NullSolution()
end

"Backward greedy search, with cycle checking."
BackwardGreedyPlanner(heuristic::Heuristic; kwargs...) =
    BackwardPlanner(;heuristic=heuristic, g_mult=0, kwargs...)

"Backward A* search."
BackwardAStarPlanner(heuristic::Heuristic; kwargs...) =
    BackwardPlanner(;heuristic=heuristic, kwargs...)
