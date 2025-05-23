# Specifications

```@meta
CurrentModule = SymbolicPlanners
```

**SymbolicPlanners.jl** provides a composable library of problem specifications
through the [`Specification`](@ref) interface, allowing users to define
a range of planning problems from modular components.

```@docs
Specification
```

Specifications support the following interface for determining goal achievement,
constraint violation, costs, rewards, and temporal discount factors.

```@docs
is_goal
is_violated
get_cost
get_reward
get_discount
```

## Goal Specifications

The most basic specifications are [`Goal`](@ref) specifications, which define
shortest path problems.

```@docs
Goal
get_goal_terms
set_goal_terms
```

The following [`Goal`](@ref) specifications are provided:

```@docs
MinStepsGoal
MinMetricGoal
MaxMetricGoal
```

It is also possible to plan to achieve a specific action using the
[`ActionGoal`](@ref) specification:

```@docs
ActionGoal
```

However, action goals are not currently supported by planners that make use 
of backward search, such as [`BackwardPlanner`](@ref) and
[`BidirectionalPlanner`](@ref).

## Constraint Specifications

**SymbolicPlanners.jl** also provides limited support for planning under
constraints. In particular, planning solutions can be constrained such that
all traversed states satisfy certain predicates:

```@docs
StateConstrainedGoal
```

Support for more general temporal constraints may be provided in the future.

## Action Costs

Many planning problems have action costs that are fixed (i.e. not state-dependent).
Problems of this sort can be defined using the following specifications:

```@docs
MinActionCosts
ExtraActionCosts
MinPerAgentActionCosts
ExtraPerAgentActionCosts
```

We also introduce interface methods to determine if a specification provides
static action costs:

```@docs
has_action_cost
get_action_cost
```

Action costs can be inferred from domains and problems using the following
utility function:

```@docs
infer_action_costs
```

## Reward Specifications

**SymbolicPlanners.jl** also provides support for goal-based reward functions
through the following specifications.

```@docs
GoalReward
BonusGoalReward
MultiGoalReward
```

Temporal discounting of rewards is supported:

```@docs
DiscountedReward
discounted
```

More general reward functions can also be defined using the
[`MaxMetricGoal`](@ref) introduced earlier, by defining a metric fluent that
is corresponds to the total reward.

Reward-based specifications should generally be used with policy-based
planning algorithms such as [`RTDP`](@ref RealTimeDynamicPlanner) and
[`MCTS`](@ref MonteCarloTreeSearch).

## Goal Simplification

It is generally useful to simplify and ground a goal specification in the
the initial state before planning, so as to reduce the runtime of checking
whether a goal is satisfied (e.g. by removing universal or existential
quantifiers). This can be achieved using the [`simplify_goal`](@ref) function:

```@docs
simplify_goal
```

To ensure a goal is simplified once before planning, and prevent further 
(potentially costly) attempts at simplification by a planner, the
[`simplified`](@ref) function can be used to create a [`SimplifiedGoal`](@ref).

```@docs
SimplifiedGoal
simplified
is_simplified
```

Preventing further simplication is useful when one planner calls another planner
internally, or when multiple planner calls with the same specification are 
expected.
