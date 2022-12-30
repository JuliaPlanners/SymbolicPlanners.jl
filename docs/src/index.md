# SymbolicPlanners.jl

A library of symbolic planners for problems and domains specified in the Planning Domain Definition Language (PDDL) and its variants. Built on top of the [PDDL.jl](https://github.com/JuliaPlanners/PDDL.jl) interface for automated planning.

## Features

- Forward state-space planning ([A*](@ref ForwardPlanner), [BFS](@ref BreadthFirstPlanner), etc.)
- [Backward](@ref BackwardPlanner) (i.e. regression) planning
- Policy-based planning ([RTDP](@ref RealTimeDynamicPlanner), [RTHS](@ref RealTimeHeuristicSearch), [MCTS](@ref MonteCarloTreeSearch), etc.)
- Relaxed-distance heuristics ([Manhattan](@ref ManhattanHeuristic), [$$h_\text{max}$$](@ref HMax), [$$h_\text{add}$$](@ref HAdd), [FF](@ref FFHeuristic), etc.)
- Policy and plan [simulation](simulators.md)
- Modular framework for [goal, reward and cost specifications](specifications.md)
- Support for [PDDL domains with numeric fluents](https://github.com/JuliaPlanners/SymbolicPlanners.jl/blob/experiments/experiments/numeric-experiments.jl) and custom datatypes

## Installation

Make sure [PDDL.jl](https://github.com/JuliaPlanners/PDDL.jl) is installed. (See [here](https://juliaplanners.github.io/PDDL.jl/dev/tutorials/getting_started/#Installation) for more instructions.)

For the stable version of SymbolicPlanners.jl, press `]` to enter the Julia package manager REPL, then run:
```
add SymbolicPlanners
```

For the latest development version, run:
```
add https://github.com/JuliaPlanners/SymbolicPlanners.jl
```

## Usage

First, [load a planning domain and problem](https://juliaplanners.github.io/PDDL.jl/dev/tutorials/getting_started/#Loading-Domains-and-Problems) using PDDL.jl. These can be loaded from local files, or from the [PlanningDomains.jl](https://github.com/JuliaPlanners/PlanningDomains.jl) online repository:

```julia
using PDDL, PlanningDomains, SymbolicPlanners

# Load Blocksworld domain and problem
domain = load_domain(:blocksworld)
problem = load_problem(:blocksworld, "problem-4")
```

Next, construct a [`Planner`](@ref), including a search [`Heuristic`](@ref) if supported, and call it on the domain and problem:

```julia
# Construct A* planner with h_add heuristic
planner = AStarPlanner(HAdd())

# Solve the problem using the planner
sol = planner(domain, problem)
```

Alternatively, we can provide planners with the initial state, and a [`Specification`](@ref) defining the goal predicates, action costs, and other desired criteria for the planning solution:

```julia
# Construct initial state from domain and problem
state = initstate(domain, problem)

# Construct goal specification that requires minimizing plan length
spec = MinStepsGoal(PDDL.get_goal(problem))

# Produce a solution given the initial state and specification
sol = planner(domain, state, spec)
```

A* search produces an ordered plan as a [`Solution`](@ref), which we can inspect and validate:

```julia-repl
julia> collect(sol)
PathSearchSolution{GenericState, Nothing}
  status: success
  plan: 10-element Vector{Term}
    (unstack b a)
    (put-down b)
    (unstack a d)
    (stack a e)
    (pick-up b)
    (stack b a)
    (pick-up c)
    (stack c b)
    (pick-up d)
    (stack d c)
  trajectory: 11-element Vector{GenericState}

julia> PDDL.satisfy(domain, sol.trajectory[end], PDDL.get_goal(problem))
true
```

We can also validate the solution by using a [`Simulator`](@ref) to determine the final state:

```julia-repl
julia> sim = EndStateSimulator(max_steps=100);

julia> end_state = sim(sol, domain, state, spec);

julia> PDDL.satisfy(domain, end_state, PDDL.get_goal(problem))
true
```
