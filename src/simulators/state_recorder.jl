export StateRecorder # Re-export from PDDL.jl

function simulate(sim::StateRecorder, sol::Solution,
 				  domain::Domain, state::State, spec::Specification)
    trajectory = [state]
    act = PDDL.no_op
	steps = sim.max_steps === nothing ? countfrom(1) : (1:sim.max_steps)
    for t in steps
		if is_goal(spec, domain, state, act) break end
        act = get_action(sol, t, state)
        state = transition(domain, state, act)
        push!(trajectory, state)
    end
    return trajectory
end
