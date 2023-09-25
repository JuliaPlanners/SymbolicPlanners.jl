export RewardAccumulator

"""
	RewardAccumulator(max_steps::Union{Int,Nothing} = nothing)

Simulator that returns accumulated reward. A specification should be provided
when using this simulator.
"""
@kwdef struct RewardAccumulator <: Simulator
	max_steps::Union{Int,Nothing} = nothing
end

function simulate(sim::RewardAccumulator,
				  domain::Domain, state::State, actions,
				  spec::Specification = NullSpecification())
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
	act = PDDL.no_op
	steps = sim.max_steps === nothing ? countfrom(1) : (1:sim.max_steps)
    for t in steps
		if is_goal(spec, domain, state, act) break end
        act = get_action(sol, t, state)
        next_state = transition(domain, state, act)
		reward += discount * get_reward(spec, domain, state, act, next_state)
		discount *= get_discount(spec)
		state = next_state
    end
    return reward
end
