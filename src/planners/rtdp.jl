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
               domain::Domain, state::State, spec::Specification)
    # Intialize then refine solution
    sol = PolicyValue()
    sol.V[hash(state)] = -planner.heuristic(domain, state, spec)
    sol = solve!(planner, sol, domain, state, spec)
    # Wrap in Boltzmann policy if needed
    return planner.action_noise == 0 ?
        sol : BoltzmannPolicy(sol, planner.action_noise)
end

function solve!(planner::RealTimeDynamicPlanner, sol::PolicyValue,
                domain::Domain, state::State, spec::Specification)
    @unpack heuristic, discount, action_noise = planner
    @unpack n_rollouts, rollout_depth, rollout_noise = planner
    # Value update subroutine
    function update!(sol, s)
        actions = available(s, domain)
        s_id = hash(s)
        if is_goal(spec, domain, state)
            qs = zeros(length(actions))
            sol.Q[s_id] = Dict{Term,Float64}(zip(actions, qs))
            sol.V[s_id] = 0.0
            return
        end
        qs = map(actions) do act
            next_s = transition(domain, s, act)
            reward = get_reward(spec, domain, s, act, next_s)
            h_val = heuristic(domain, next_s, spec)
            return discount * get!(sol.V, hash(next_s), -h_val) + reward
        end
        sol.Q[s_id] = Dict{Term,Float64}(zip(actions, qs))
        sol.V[s_id] = action_noise == 0 ?
            maximum(qs) : sum(softmax(qs ./ action_noise) .* qs)
    end
    # Perform rollouts from initial state
    initial_state = state
    ro_policy = rollout_noise == 0 ? sol : BoltzmannPolicy(sol, rollout_noise)
    visited = State[]
    for n in 1:n_rollouts
        state = initial_state
        # Rollout until maximum depth
        for t in 1:rollout_depth
            push!(visited, state)
            if is_goal(spec, domain, state) break end
            update!(sol, state)
            act = rand_action(ro_policy, state)
            state = transition(domain, state, act)
        end
        # Post-rollout update
        while length(visited) > 0
            state = pop!(visited)
            update!(sol, state)
        end
    end
    return sol
end

function solve!(planner::RealTimeDynamicPlanner,
                sol::BoltzmannPolicy{PolicyValue},
                domain::Domain, state::State, spec::Specification)
    sol = solve!(planner, sol.policy, domain, state, spec)
    return BoltzmannPolicy(sol, planner.action_noise)
end
