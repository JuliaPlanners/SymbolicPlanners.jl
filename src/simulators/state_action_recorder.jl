export StateActionRecorder

"""
	StateActionRecorder(max_steps::Union{Int,Nothing} = nothing)

Simulator that records the state-action trajectory, including the start state.
"""
@kwdef struct StateActionRecorder <: Simulator
	max_steps::Union{Int,Nothing} = nothing
end

function simulate(sim::StateActionRecorder,
				  domain::Domain, state::State, actions)
    actions = Term[]
    trajectory = State[state]
	for (t, act) in enumerate(actions)
        state = transition(domain, state, act)
		push!(actions, act)
		push!(trajectory, state)
		sim.max_steps !== nothing && t >= sim.max_steps && break
    end
	return (actions, trajectory)
end

function simulate(sim::StateActionRecorder, sol::Solution,
 				  domain::Domain, state::State, spec::Specification)
    actions = Term[]
    trajectory = [state]
	steps = sim.max_steps === nothing ? countfrom(1) : (1:sim.max_steps)
    for t in steps
		if is_goal(spec, domain, state) break end
        act = get_action(sol, t, state)
        state = transition(domain, state, act)
		push!(actions, act)
        push!(trajectory, state)
    end
    return (actions, trajectory)
end
