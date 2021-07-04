export Specification, Goal

"Abstract specification for a planning problem"
abstract type Specification end

"Check if `state` is a goal state according to the specification."
is_goal(spec::Specification, domain::Domain, state::State) =
    error("Not implemented.")

"Check if `state` violates specified constraints."
is_violated(spec::Specification, domain::Domain, state::State) =
    error("Not implemented.")

"Returns the cost of going from state `s1` to state `s2` via action `a`."
get_cost(spec::Specification, domain::Domain, s1::State, a::Term, s2::State) =
    error("Not implemented.")

"Returns the reward of going from state `s1` to state `s2` via action `a`."
get_reward(spec::Specification, domain::Domain, s1::State, a::Term, s2::State) =
    error("Not implemented.")

"Abstract type for goal-based specifications."
abstract type Goal <: Specification end

"Return goal terms."
get_goal_terms(spec::Goal) =
    error("Not implemented.")

include("specifications/min_steps.jl")
include("specifications/min_metric.jl")
include("specifications/state_constrained.jl")

# Convenience constructors
Specification(problem::Problem) = problem.metric === nothing ?
    MinStepsGoal(problem) : MinMetricGoal(problem)
Specification(goals::AbstractVector{<:Term}) = MinStepsGoal(goals)
Specification(goal::Term) = MinStepsGoal(goal)
