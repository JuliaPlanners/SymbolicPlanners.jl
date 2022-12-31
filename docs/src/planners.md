# Planners

```@meta
CurrentModule = SymbolicPlanners
```

The core of SymbolicPlanners.jl is a library of planning algorithms, or 
[`Planner`](@ref)s:

```@docs
Planner
```

[`Planner`](@ref)s define the following interface:

```@docs
solve
refine!
refine
```

## Breadth-First Search

As a baseline uninformed planning algorithm, one can use 
[`BreadthFirstPlanner`](@ref):

```@docs
BreadthFirstPlanner
```

## Forward Heuristic Search

The modern day workhorse for automated planning is forward heuristic search,
implemented by [`ForwardPlanner`](@ref).

```@docs
ForwardPlanner
```

Several convenience constructors for variants of [`ForwardPlanner`](@ref)
are provided:

```@docs
UniformCostPlanner
GreedyPlanner
AStarPlanner
WeightedAStarPlanner
```

Probabilistic variants of forward search are also provided:

```@docs
ProbForwardPlanner
ProbAStarPlanner
```

## Backward Heuristic Search

Due to the flexibility of the PDDL.jl API, SymbolicPlanners.jl supports
backward search:

```@docs
BackwardPlanner
```

The following convenience constructors are provided:

```@docs
BackwardGreedyPlanner
BackwardAStarPlanner
```

As with [`ForwardPlanner`](@ref), probabilistic variants of backward search
are provided:

```@docs
ProbBackwardPlanner
ProbBackwardAStarPlanner
```

## Bidirectional Search

A simple implementation of bidirectional search is provided by 
[`BidirectionalPlanner`](@ref).

```@docs
BidirectionalPlanner
```

The following convenience constructors are provided:

```@docs
BiGreedyPlanner
BiAStarPlanner
```

## Policy-Based Planners

Unlike most systems for automated symbolic planning, SymbolicPlanners.jl
includes several policy-based planners which return [`PolicySolution`](@ref)s.

These planners are especially useful for stochastic environments (even if the
planner operates over a determinized version of the true domain), since they
compute what action should be performed in each encountered state, rather than
just a sequence of ordered actions. They are also useful for real-time planning
via reuse of previous solutions.

```@docs
RealTimeDynamicPlanner
RealTimeHeuristicSearch
MonteCarloTreeSearch
```

## External Planners

Wrappers for several external planners are provided:

```@docs
FastDownward
Pyperplan
ENHSP
```