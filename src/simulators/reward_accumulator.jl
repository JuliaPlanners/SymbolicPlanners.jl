export RewardAccumulator

"Simulator that returns accumulated reward."
@kwdef struct RewardAccumulator <: Simulator
	max_steps::Union{Int,Nothing} = nothing
end

function simulate(sim::RewardAccumulator,
				  domain::Domain, state::State, actions)
    reward = 0.0
    discount = get_discount(spec)
	for (t, act) in enumerate(actions)
		next_state = transition(domain, state, act)
		reward += discount * get_reward(spec, domain, state, act, next_state)
		discount *= get_discount(spec)
		state = next_state
		sim.max_steps !== nothing && t >= sim.max_steps && break
    end
	return reward
end

function simulate(sim::RewardAccumulator, sol::Solution,
				  domain::Domain, state::State, spec::Specification)
	reward = 0.0
	discount = get_discount(spec)
	steps = sim.max_steps === nothing ? countfrom(1) : (1:sim.max_steps)
    for t in steps
		if is_goal(spec, domain, state) break end
        act = get_action(sol, t, state)
        next_state = transition(domain, state, act)
		reward += discount * get_reward(spec, domain, state, act, next_state)
		discount *= get_discount(spec)
		state = next_state
    end
    return reward
end
