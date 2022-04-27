export RealTimeDynamicPlanner, RTDP

"Planner that uses Real Time Dynamic Programming (RTDP)."
@kwdef mutable struct RealTimeDynamicPlanner <: Planner
    heuristic::Heuristic = GoalCountHeuristic()
    n_rollouts::Int = 50
    max_depth::Int = 50
    rollout_noise::Float64 = 0.0
    action_noise::Float64 = 0.0
end

const RTDP = RealTimeDynamicPlanner

function solve(planner::RealTimeDynamicPlanner,
               domain::Domain, state::State, spec::Specification)
    # Precompute heuristic information
    ensure_precomputed!(planner.heuristic, domain, state, spec)
    # Initialize then refine solution
    default = FunctionalVPolicy(planner.heuristic, domain, spec)
    sol = TabularPolicy(default=default)
    sol.V[hash(state)] = -compute(planner.heuristic, domain, state, spec)
    sol = solve!(planner, sol, domain, state, spec)
    # Wrap in Boltzmann policy if needed
    return planner.action_noise == 0 ?
        sol : BoltzmannPolicy(sol, planner.action_noise)
end

function solve!(planner::RealTimeDynamicPlanner, sol::TabularPolicy,
                domain::Domain, state::State, spec::Specification)
    @unpack heuristic, action_noise = planner
    @unpack n_rollouts, max_depth, rollout_noise = planner
    # Precompute heuristic information
    ensure_precomputed!(heuristic, domain, state, spec)
    # Perform rollouts from initial state
    initial_state = state
    ro_policy = rollout_noise == 0 ? sol : BoltzmannPolicy(sol, rollout_noise)
    visited = Vector{typeof(state)}()
    for n in 1:n_rollouts
        state = initial_state
        # Rollout until maximum depth
        for t in 1:max_depth
            push!(visited, state)
            if is_goal(spec, domain, state) break end
            update_values!(planner, sol, domain, state, spec)
            act = get_action(ro_policy, state)
            state = transition(domain, state, act)
        end
        # Post-rollout update
        while length(visited) > 0
            state = pop!(visited)
            update_values!(planner, sol, domain, state, spec)
        end
    end
    return sol
end

function solve!(planner::RealTimeDynamicPlanner,
                sol::BoltzmannPolicy{TabularPolicy},
                domain::Domain, state::State, spec::Specification)
    sol = solve!(planner, sol.policy, domain, state, spec)
    return BoltzmannPolicy(sol, planner.action_noise)
end

function update_values!(planner::RealTimeDynamicPlanner, sol::TabularPolicy,
                        domain::Domain, state::State, spec::Specification)
    actions = collect(available(domain, state))
    state_id = hash(state)
    if is_goal(spec, domain, state)
        qs = zeros(length(actions))
        sol.Q[state_id] = Dict{Term,Float64}(a => 0 for a in actions)
        sol.V[state_id] = 0.0
        return
    end
    qs = map(actions) do act
        next_state = transition(domain, state, act)
        next_v = get!(sol.V, hash(next_state)) do
            -compute(planner.heuristic, domain, next_state, spec)
        end
        r = get_reward(spec, domain, state, act, next_state)
        return get_discount(spec) * next_v + r
    end
    sol.Q[state_id] = Dict{Term,Float64}(zip(actions, qs))
    sol.V[state_id] = planner.action_noise == 0 ?
        maximum(qs) : sum(softmax(qs ./ planner.action_noise) .* qs)
end
