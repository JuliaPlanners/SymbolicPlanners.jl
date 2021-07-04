export Simulator
export EndStateSimulator, RewardAccumulator, StateActionRecorder

"Abstract type for simulators."
abstract type Simulator end

(sim::Simulator)(sol::Solution, domain::Domain, state::State, spec::Specification) =
    run_simulator(sim, sol, domain, state, spec)
(sim::Simulator)(sol::Solution, domain::Domain, state::State, spec) =
	sim(sol, domain, state, Specification(spec))
(sim::Simulator)(sol::Solution, domain::Domain, state::State) =
    sim(sol, domain, state, NullSpecification())

"Simulator that returns end state of simulation."
@kwdef struct EndStateSimulator <: Simulator
    max_steps::Int = typemax(Int)
end

function run_simulator(sim::EndStateSimulator, sol::Solution,
					   domain::Domain, state::State, spec::Specification)
    for t in 1:sim.max_steps
		if is_goal(spec, domain, state) break end
        act = get_action(sol, t, state)
        state = transition(domain, state, act)
    end
    return state
end

"Simulator that returns accumulated reward."
@kwdef struct RewardAccumulator <: Simulator
    max_steps::Int = typemax(Int)
end

function run_simulator(sim::RewardAccumulator, sol::Solution,
					   domain::Domain, state::State, spec::Specification)
	reward = 0.0
	discount = get_discount(spec)
    for t in 1:sim.max_steps
		if is_goal(spec, domain, state) break end
        act = get_action(sol, t, state)
        next_state = transition(domain, state, act)
		reward += discount * get_reward(spec, domain, state, act, next_state)
		discount *= get_discount(spec)
		state = next_state
    end
    return reward
end

"Simulator that records the state-action trajectory, including the start state."
@kwdef struct StateActionRecorder <: Simulator
    max_steps::Int = typemax(Int)
end

function run_simulator(sim::StateActionRecorder, sol::Solution,
 					   domain::Domain, state::State, spec::Specification)
    actions = Term[]
    trajectory = State[state]
    for t in 1:sim.max_steps
		if is_goal(spec, domain, state) break end
        act = get_action(sol, t, state)
        state = transition(domain, state, act)
		push!(actions, act)
        push!(trajectory, state)
    end
    return (actions, trajectory)
end
