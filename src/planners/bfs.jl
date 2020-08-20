export BreadthFirstPlanner

"Uninformed breadth-first search planner."
@kwdef mutable struct BreadthFirstPlanner <: Planner
    max_nodes::Real = Inf
end

set_max_resource(planner::BreadthFirstPlanner, val) =
    @set planner.max_nodes = val

"Uninformed breadth-first search for a plan."
function call(planner::BreadthFirstPlanner,
              domain::Domain, state::State, goal_spec::GoalSpec)
    @unpack goals, constraints = goal_spec
    # Initialize backpointers and queue
    state_hash = hash(state)
    state_dict = Dict{UInt,State}(state_hash => state)
    parents = Dict{UInt,Tuple{UInt,Term}}()
    queue = [state_hash]
    count = 1
    while length(queue) > 0
        # Pop state off the queue
        state_hash = popfirst!(queue)
        state = state_dict[state_hash]
        # Return plan if max depth is reached or goals are satisfied
        if count >= planner.max_nodes || satisfy(goals, state, domain)[1]
            plan, traj = extract_plan(state_hash, state_dict, parents)
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
            if haskey(state_dict, next_hash) continue end
            state_dict[next_hash] = next_state
            # Check if next state satisfies trajectory constraints
            if !isempty(constraints) && !next_state[domain, constraints]
                continue end
            # Update backpointer and add next state to queue
            parents[next_hash] = (state_hash, act)
            push!(queue, next_hash)
        end
    end
    return NullSolution()
end
