using Test, Random
using PDDL, PlanningDomains
using SymbolicPlanners

# Register array theory for gridworld domains
PDDL.Arrays.register!()

# Load domains and problems
gridworld = load_domain(:gridworld)
gw_problem = load_problem(:gridworld, "problem-1")
gw_state = initstate(gridworld, gw_problem)
gw_spec = Specification(gw_problem)

doors_keys_gems = load_domain(:doors_keys_gems)
dkg_problem = load_problem(:doors_keys_gems, "problem-1")
dkg_state = initstate(doors_keys_gems, dkg_problem)
dkg_spec = Specification(dkg_problem)

blocksworld = load_domain(:blocksworld)
bw_problem = load_problem(:blocksworld, "problem-2")
bw_state = initstate(blocksworld, bw_problem)
bw_spec = Specification(bw_problem)

bw_axioms = load_domain(:blocksworld_axioms)
ba_problem = load_problem(:blocksworld_axioms, "problem-2")
ba_state = initstate(bw_axioms, ba_problem)
ba_spec = Specification(ba_problem)

zeno_travel = load_domain(:zeno_travel)
zt_problem = load_problem(:zeno_travel, "problem-2")
zt_state = initstate(zeno_travel, zt_problem)
zt_spec = MinStepsGoal(zt_problem)

# Test heuristics
include("heuristics.jl")

# Test planners
include("planners.jl")

# Test problem with constraints
include("cabbage.jl")

PDDL.Arrays.deregister!()
