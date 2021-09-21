using Test, Random
using PDDL, SymbolicPlanners

# Register array theory for gridworld domains
PDDL.Arrays.register!()

include("heuristics.jl")
include("planners.jl")

PDDL.Arrays.deregister!()
