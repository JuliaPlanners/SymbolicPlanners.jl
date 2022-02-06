export Simulator

import PDDL: simulate

"""
	(sim::Simulator)(sol::Solution, domain::Domain, state::State, [spec])

Simulates the execution of `sol in a PDDL `domain` starting from an initial
`state`. A goal specification `spec` may be provided to serve as a terminating
condition for the simulation.
"""
(sim::Simulator)(sol::Solution, domain::Domain, state::State, spec::Specification) =
    simulate(sim, sol, domain, state, spec)
(sim::Simulator)(sol::Solution, domain::Domain, state::State, spec) =
	sim(sol, domain, state, Specification(spec))
(sim::Simulator)(sol::Solution, domain::Domain, state::State) =
    sim(sol, domain, state, NullSpecification())

include("end_state.jl")
include("state_recorder.jl")
include("state_action_recorder.jl")
include("reward_accumulator.jl")
