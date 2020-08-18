# SymbolicPlanners.jl

Symbolic planners for problems and domains specified in [PDDL](https://github.com/JuliaPlanners/PDDL.jl).

## Installation

Make sure [`PDDL.jl`](https://github.com/JuliaPlanners/PDDL.jl) is installed. Then run
```
add https://github.com/JuliaPlanners/SymbolicPlanners.jl
```
at the Julia package manager.

## Features

- Forward state-space planners (A*, BFS, etc.)
- Backward (i.e. regression) planners
- Relaxed-distance heuristics (Manhattan, _h_<sub>add</sub>, _h_<sub>max</sub>, etc.)

## Planners

- [Forward breadth-first search](src/planners/bfs.jl)
- [Forward A* search](src/planners/astar.jl)
- [Backward (A*) search](src/planners/backward.jl)
- [FastDownward wrapper](src/planners/external.jl)

## Heuristics

- Goal Count: counts the number of unsatisfied goals
- Manhattan: L<sub>1</sub> distance for arbitrary numeric fluents
- HSP heuristics: _h_<sub>add</sub>, _h_<sub>max</sub>, etc.
- HSPr heuristics: the above, but for backward search
