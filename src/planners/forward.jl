export ForwardPlanner, BestFirstPlanner, UniformCostPlanner, GreedyPlanner
export AStarPlanner, WeightedAStarPlanner

"Forward best-first search planner."
@kwdef struct ForwardPlanner <: Planner
    heuristic::Heuristic = GoalCountHeuristic()
    g_mult::Real = 1 # Path cost multiplier
    h_mult::Real = 1 # Heuristic multiplier
    max_nodes::Real = Inf
end

set_max_resource(planner::ForwardPlanner, val) =
    @set planner.max_nodes = val

"Deterministic best-first search for a plan."
function call(planner::ForwardPlanner,
              domain::Domain, state::State, goal_spec::GoalSpec)
    @unpack goals, metric, constraints = goal_spec
    @unpack max_nodes, g_mult, h_mult, heuristic = planner
    # Perform any precomputation required by the heuristic
    heuristic = precompute!(heuristic, domain, state, goal_spec)
    # Initialize path costs and priority queue
    state_hash = hash(state)
    state_dict = Dict{UInt,State}(state_hash => state)
    parents = Dict{UInt,Tuple{UInt,Term}}()
    path_costs = Dict{UInt,Float64}(state_hash => 0)
    est_cost = h_mult * heuristic(domain, state, goal_spec)
    queue = PriorityQueue{UInt,Float64}(state_hash => est_cost)
    count = 1
    while length(queue) > 0
        # Get state with lowest estimated cost to goal
        state_hash = dequeue!(queue)
        state = state_dict[state_hash]
        # Return plan if search budget is reached or goals are satisfied
        if count >= max_nodes || satisfy(goals, state, domain)[1]
            return reconstruct_plan(state_hash, state_dict, parents) end
        count += 1
        # Get list of available actions
        actions = available(state, domain)
        # Iterate over actions
        for act in actions
            # Execute action and trigger all post-action events
            next_state = transition(domain, state, act; check=false)
            next_hash = hash(next_state)
            # Check if next state satisfies trajectory constraints
            if !isempty(constraints) && !next_state[domain, constraints]
                continue end
            # Compute path cost
            act_cost = metric == nothing ? 1 :
                next_state[domain, metric] - state[domain, metric]
            path_cost = path_costs[state_hash] + act_cost
            # Update path costs if new path is shorter
            cost_diff = get(path_costs, next_hash, Inf) - path_cost
            if cost_diff > 0
                if !(next_hash in keys(state_dict))
                    state_dict[next_hash] = next_state end
                parents[next_hash] = (state_hash, act)
                path_costs[next_hash] = path_cost
                # Update estimated cost from next state to goal
                if !(next_hash in keys(queue))
                    g_val = g_mult * path_cost
                    h_val = h_mult * heuristic(domain, next_state, goal_spec)
                    enqueue!(queue, next_hash, g_val + h_val)
                else
                    queue[next_hash] -= cost_diff
                end
            end
        end
    end
    return nothing, nothing
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
