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

"Null goal specification."
struct NullGoal <: Goal end

is_goal(::NullGoal, ::Domain, ::State) = true
is_violated(::NullGoal, ::Domain, ::State) = false
get_cost(::NullGoal, ::Domain, ::State, ::Term, ::State) = 0.0
get_reward(::NullGoal, ::Domain, ::State, ::Term, ::State) = 0.0
get_discount(::NullGoal) = 1.0
get_goal_terms(::NullGoal) = Term[]

include("min_steps.jl")
include("min_metric.jl")
include("max_metric.jl")
include("state_constrained.jl")
include("action_costs.jl")
include("discounted.jl")
include("goal_reward.jl")
include("backward.jl")

# Convenience constructors
function Specification(problem::Problem)
    metric = PDDL.get_metric(problem)
    if metric === nothing
        return MinStepsGoal(problem)
    elseif metric.name == :minimize
        return MinMetricGoal(problem)
    elseif metric.name == :maximize
        return MaxMetricGoal(problem)
    else
        error("Unrecognized metric direction: $(metric.name)")
    end
end
Specification(goals::AbstractVector{<:Term}) = MinStepsGoal(goals)
Specification(goal::Term) = MinStepsGoal(goal)
