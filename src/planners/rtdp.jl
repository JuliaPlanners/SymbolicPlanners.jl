export RealTimeDynamicPlanner, RTDP

"Planner that uses Real Time Dynamic Programming (RTDP)."
@kwdef mutable struct RealTimeDynamicPlanner <: Planner
    heuristic::Heuristic = GoalCountHeuristic()
    discount::Float64 = 1.0
    action_noise::Float64 = 0.0
    n_rollouts::Int = 50
    rollout_depth::Int = 50
    rollout_noise::Float64 = 0.0
end

const RTDP = RealTimeDynamicPlanner

function solve(planner::RealTimeDynamicPlanner,
               domain::Domain, state::State, goal_spec::GoalSpec)
    # Intialize then refine solution
    sol = PolicyValueSolution()
    sol.V[hash(state)] = -planner.heuristic(domain, state, goal_spec)
    return solve!(planner, sol, domain, state, goal_spec)
end

function solve!(planner::RealTimeDynamicPlanner, sol::PolicyValueSolution,
                domain::Domain, state::State, goal_spec::GoalSpec)
    @unpack goals, metric = goal_spec
    @unpack heuristic, discount, action_noise = planner
    @unpack n_rollouts, rollout_depth, rollout_noise = planner
    # Value update subroutine
    function update!(sol, s)
        actions = available(s, domain)
        s_id = hash(s)
        if satisfy(goals, state, domain)[1]
            qs = zeros(length(actions))
            sol.Q[s_id] = Dict{Term,Float64}(zip(actions, qs))
            sol.V[s_id] = 0.0
            return
        end
        qs = map(actions) do act
            next_s = transition(domain, s, act)
            act_cost = metric === nothing ? 1 :
                next_s[domain, metric] - s[domain, metric]
            h_val = heuristic(domain, next_s, goal_spec)
            return discount * get!(sol.V, hash(next_s), -h_val) - act_cost
        end
        sol.Q[s_id] = Dict{Term,Float64}(zip(actions, qs))
        sol.V[s_id] = action_noise == 0 ?
            maximum(qs) : sum(softmax(qs ./ action_noise) .* qs)
    end
    # Perform rollouts from initial state
    initial_state = state
    sol.action_noise = rollout_noise
    visited = State[]
    for n in 1:n_rollouts
        state = initial_state
        # Rollout until maximum depth
        for t in 1:rollout_depth
            push!(visited, state)
            if satisfy(goals, state, domain)[1] break end
            update!(sol, state)
            act = rand_action(sol, state)
            state = transition(domain, state, act)
        end
        # Post-rollout update
        while length(visited) > 0
            state = pop!(visited)
            update!(sol, state)
        end
    end

    # Update action noise and return solution
    sol.action_noise = action_noise
    return sol
end
