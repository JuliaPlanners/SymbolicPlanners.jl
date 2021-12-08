export EndStateSimulator # Re-export from PDDL.jl

function simulate(sim::EndStateSimulator, sol::Solution,
				  domain::Domain, state::State, spec::Specification)
	steps = sim.max_steps === nothing ? countfrom(1) : (1:sim.max_steps)
	state = copy(state)
    for t in steps
		if is_goal(spec, domain, state) break end
        act = get_action(sol, t, state)
        state = transition!(domain, state, act)
    end
    return state
end
