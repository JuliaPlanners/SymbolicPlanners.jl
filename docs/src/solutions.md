# Solutions

```@meta
CurrentModule = SymbolicPlanners
```

Planners in SymbolicPlanners.jl return [`Solution`](@ref)s, which represent
(potentially partial) descriptions of how a problem should be solved.

```@docs
Solution
```

```@docs
get_action(::Solution, ::Int, ::State)
```

In the case that a problem is unsolved or unsolvable, planners may return a
[`NullSolution`](@ref):

```@docs
NullSolution
```

## Ordered Solutions

One class of solutions returned by planners are [`OrderedSolution`]s(@ref),
which define an ordered sequence of actions (i.e. a plan) that must be taken
to reach a goal.

```@docs
OrderedSolution
get_action(::OrderedSolution, ::Int)
```

```@docs
OrderedPlan
```

## Path Search Solutions

A particular type of [`OrderedSolution`](@ref) is returned by search-based
shortest-path planners (i.e. [`BreadthFirstPlanner`](@ref),
[`ForwardPlanner`](@ref), and [`BackwardPlanner`](@ref)). These
[`PathSearchSolution`](@ref)s may store information about the search process in
addition to the discovered plan, allowing such information to be used by future
searches (e.g. through calls to [`refine!`](@ref)).

```@docs
PathSearchSolution
```

Bidirectional search-based planners also have a corresponding solution type:

```@docs
BiPathSearchSolution
```

Nodes in a search tree have the following type:

```@docs
PathNode
```

## Policy Solutions

Another important class of solutions are [`PolicySolution`](@ref)s, which 
specify the action to be taken given a particular state. This is especially
useful when the true environment is stochastic, such that agents may end up
in a state that is different than expected, or when it is desirable to reuse
solutions from one initial state in a different initial state.

```@docs
PolicySolution
```

The following methods constitute the interface for [`PolicySolution`](@ref)s:

```@docs
get_action(::PolicySolution, ::State)
best_action
rand_action
get_value
get_action_values
get_action_probs
get_action_prob
```

A [`NullPolicy`](@ref) can be used as a default when no information is known.

```@docs
NullPolicy
```

## Deterministic Policies

SymbolicPlanners.jl provides the following deterministic policies, i.e., 
policies that always return the estimated best action for a given state:

```@docs
TabularPolicy
TabularVPolicy
FunctionalVPolicy
HeuristicVPolicy
ReusableTreePolicy
```

## Stochastic Policies

SymbolicPlanners.jl also provides stochastic policies, some of which are 
intended for use as wrappers around deterministic policies:

```@docs
RandomPolicy
EpsilonGreedyPolicy
BoltzmannPolicy
BoltzmannMixturePolicy
MixturePolicy
```
