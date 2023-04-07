# Heuristics

```@meta
CurrentModule = SymbolicPlanners
```

SymbolicPlanners.jl provides a library of search heuristics which estimate the 
distance between a state and the goal.

```@docs
Heuristic
```

[`Heuristic`](@ref)s define the following interface:

```@docs
compute
precompute!
is_precomputed
ensure_precomputed!
```

## Basic Heuristics

Several basic heuristics are provided:

```@docs
NullHeuristic
GoalCountHeuristic
ReachabilityHeuristic
```

These heuristics are general but not very informative. As such, they are best
used as baselines, or for testing the correctness of planning algorithms on
small problems.

## Metric Heuristics

In domains with numeric fluents, distance metrics can be used as heuristics
for goals with equality constraints:

```@docs
MetricHeuristic
ManhattanHeuristic
EuclideanHeuristic
```

## Relaxed Planning Graph Heuristics

Several relaxed planning graph heuristics are provided by SymbolicPlanners.jl.
In contrast to most other planning systems, these implementations also support 
domains with non-Boolean fluents.

```@docs
HSPHeuristic
HMax
HAdd
FFHeuristic
```

## Backward Search Heuristics

A few relaxed planning graph heuristics are also provided for backward search:

```@docs
HSPRHeuristic
HMaxR
HAddR
```

## Wrapper Heuristics

Since plans and policies can be used to estimate the cost of achieving a goal, 
the following wrapper heuristics are provided.

```@docs
PlannerHeuristic
PolicyValueHeuristic
GoalDependentPolicyHeuristic
```

## Precomputation and Memoization

Some applications of planning algorithms require repeated calls to a planner
(e.g. in [Bayesian inverse planning](https://dl.acm.org/doi/abs/10.5555/3495724.3497338)),
which may lead to repeated pre-computation of the search heuristic. In such
cases, overhead can be substantially reduced by ensuring that precomputation
happens only once. This can be done using the [`precomputed`](@ref)
function to construct a [`PrecomputedHeuristic`](@ref).

```@docs
precomputed
PrecomputedHeuristic
```

Similarly, if heuristic computation is costly, memoization of heuristic values
can lead to faster results. This can be achieved using the [`memoized`](@ref)
function to construct a [`MemoizedHeuristic`](@ref)

```@docs
memoized
MemoizedHeuristic
```

In cases where both precomputation and memoization are desired, users should
perform precomputation before memoization:

```julia
heuristic = memoized(precomputed(heuristic))
```
