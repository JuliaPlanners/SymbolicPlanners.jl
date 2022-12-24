export RealTimeDynamicPlanner, RTDP

"Planner that uses Real Time Dynamic Programming (RTDP)."
@kwdef mutable struct RealTimeDynamicPlanner <: Planner
    heuristic::Heuristic = GoalCountHeuristic()
    n_rollouts::Int = 50
    max_depth::Int = 50
    rollout_noise::Float64 = 0.0
    action_noise::Float64 = 0.0
end

@auto_hash RealTimeDynamicPlanner
@auto_equals RealTimeDynamicPlanner

const RTDP = RealTimeDynamicPlanner

function Base.copy(p::RealTimeDynamicPlanner)
    return RealTimeDynamicPlanner(p.heuristic, p.n_rollouts, p.max_depth,
                                  p.rollout_noise, p.action_noise)
end

function solve(planner::RealTimeDynamicPlanner,
               domain::Domain, state::State, spec::Specification)
    # Simplify goal specification
    spec = simplify_goal(spec, domain, state)
    # Precompute heuristic information
    precompute!(planner.heuristic, domain, state, spec)
    # Initialize then refine solution
    default = FunctionalVPolicy(planner.heuristic, domain, spec)
    sol = TabularPolicy(default)
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
    # Simplify goal specification
    spec = simplify_goal(spec, domain, state)
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
    @unpack action_noise = planner
    state_id = hash(state)
    qs = get!(Dict{Term,Float64}, sol.Q, state_id)
    if is_goal(spec, domain, state)
        # Value of goal / terminal state is zero
        for act in available(domain, state)
            qs[act] = 0.0
        end
        sol.V[state_id] = 0.0
    else
        # Back-propogate values from successor states
        for act in available(domain, state)
            next_state = transition(domain, state, act)
            next_v = get!(sol.V, hash(next_state)) do
                -compute(planner.heuristic, domain, next_state, spec)
            end
            r = get_reward(spec, domain, state, act, next_state)
            qs[act] =  get_discount(spec) * next_v + r
        end
        if action_noise == 0
            sol.V[state_id] = maximum(values(qs))
        else
            qvals = collect(values(qs))
            sol.V[state_id] = sum(softmax(qvals ./ action_noise) .* qvals)
        end
    end
    return nothing
end
