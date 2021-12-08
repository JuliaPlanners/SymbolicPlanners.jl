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

include("simulators/end_state.jl")
include("simulators/state_recorder.jl")
include("simulators/state_action_recorder.jl")
include("simulators/reward_accumulator.jl")
