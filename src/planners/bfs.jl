export BreadthFirstPlanner

"Uninformed breadth-first search planner."
@kwdef mutable struct BreadthFirstPlanner <: Planner
    max_nodes::Int = typemax(Int)
end

set_max_resource(planner::BreadthFirstPlanner, val) =
    @set planner.max_nodes = val

"Uninformed breadth-first search for a plan."
function call(planner::BreadthFirstPlanner,
              domain::Domain, state::State, goal_spec::GoalSpec)
    @unpack goals, constraints = goal_spec
    # Initialize backpointers and queue
    state_hash = hash(state)
    search_tree = Dict{UInt,SearchNode}(state_hash => SearchNode(state, 0))
    queue = [state_hash]
    count = 1
    while length(queue) > 0
        # Pop state off the queue
        state_hash = popfirst!(queue)
        state = search_tree[state_hash].state
        # Return plan if max depth is reached or goals are satisfied
        if count >= planner.max_nodes || satisfy(goals, state, domain)[1]
            plan, traj = reconstruct(state_hash, search_tree)
            return BasicSolution(plan, traj)
        end
        count += 1
        # Iterate over available actions
        actions = available(state, domain)
        for act in actions
            # Execute actions on state
            next_state = transition(domain, state, act; check=false)
            next_hash = hash(next_state)
            # Skip if state has already been encountered
            if haskey(search_tree, next_hash) continue end
            # Check if next state satisfies trajectory constraints
            if !isempty(constraints) && !next_state[domain, constraints]
                continue end
            # Update backpointer and add next state to queue
            search_tree[next_hash] = SearchNode(next_state, 0, state_hash, act)
            push!(queue, next_hash)
        end
    end
    return NullSolution()
end
