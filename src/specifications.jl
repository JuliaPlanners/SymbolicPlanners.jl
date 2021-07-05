export Specification, Goal
export is_goal, is_violated, get_cost, get_reward

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

"Returns the reward discount factor."
get_discount(spec::Specification) =
    error("Not implemented.")

"Null specification."
struct NullSpecification <: Specification end

is_goal(::NullSpecification, ::Domain, ::State) = false
is_violated(::NullSpecification, ::Domain, ::State) = false
get_cost(::NullSpecification, ::Domain, ::State, ::Term, ::State) = 0.0
get_reward(::NullSpecification, ::Domain, ::State, ::Term, ::State) = 0.0
get_discount(::NullSpecification) = 1.0

"Abstract type for goal-based specifications."
abstract type Goal <: Specification end

"Return goal terms."
get_goal_terms(spec::Goal) = error("Not implemented.")

# No discounting for goal specifications by default
get_discount(spec::Goal) = 1.0

include("specifications/min_steps.jl")
include("specifications/min_metric.jl")
include("specifications/max_metric.jl")
include("specifications/state_constrained.jl")
include("specifications/discounted.jl")
include("specifications/goal_reward.jl")

# Convenience constructors
Specification(problem::Problem) = problem.metric === nothing ?
    MinStepsGoal(problem) : Specification(problem)
Specification(goals::AbstractVector{<:Term}) = MinStepsGoal(goals)
Specification(goal::Term) = MinStepsGoal(goal)
