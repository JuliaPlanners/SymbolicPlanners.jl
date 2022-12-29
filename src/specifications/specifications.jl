export Specification, Goal
export is_goal, is_violated, get_cost, get_reward

"""
    $(TYPEDEF)

Abstract type for problem specifications, which can define goal predicates,
action costs, reward functions, and other desired criteria for planning
[`Solution`](@ref)s.
"""
abstract type Specification end

"""
$(SIGNATURES)

Check if `state` is a goal state according to the specification.
"""
is_goal(spec::Specification, domain::Domain, state::State) =
    error("Not implemented.")

"""
$(SIGNATURES)

Check if `state` violates specified constraints.
"""
is_violated(spec::Specification, domain::Domain, state::State) =
    error("Not implemented.")

"""
$(SIGNATURES)

Returns the cost of going from state `s1` to state `s2` via action `a`.
"""
get_cost(spec::Specification, domain::Domain, s1::State, a::Term, s2::State) =
    error("Not implemented.")

"""
$(SIGNATURES)

Returns the reward of going from state `s1` to state `s2` via action `a`.
"""
get_reward(spec::Specification, domain::Domain, s1::State, a::Term, s2::State) =
    error("Not implemented.")

"""
$(SIGNATURES)

Returns the reward discount factor.
"""
get_discount(spec::Specification) =
    error("Not implemented.")

"""
    NullSpecification()

Null specification, which can never be satisfied, as no constraints, and has
zero costs or rewards.
"""
struct NullSpecification <: Specification end

is_goal(::NullSpecification, ::Domain, ::State) = false
is_violated(::NullSpecification, ::Domain, ::State) = false
get_cost(::NullSpecification, ::Domain, ::State, ::Term, ::State) = 0.0
get_reward(::NullSpecification, ::Domain, ::State, ::Term, ::State) = 0.0
get_discount(::NullSpecification) = 1.0

"""
    $(TYPEDEF)

Abstract type for goal-based specifications, which define a shortest path
problem to a set of goal states. The set of goal states is typically defined
by a list of terms that must hold true for the goal to be satisfied.

In the context of Markov decision processes, a goal state is a terminal state.
If all actions also have positive cost (i.e. negative reward), this constitutes
a stochastic shortest path problem.
"""
abstract type Goal <: Specification end

"""
$(SIGNATURES)

Return goal terms.
"""
get_goal_terms(spec::Goal) = error("Not implemented.")

"""
$(SIGNATURES)

Return a copy of the goal specification with updated goal terms.
"""
set_goal_terms(spec::Goal, terms) = error("Not implemented.")

# No discounting for goal specifications by default
get_discount(spec::Goal) = 1.0

"""
    NullGoal()

Null goal specification, with no terms to be satisfied (i.e. every state
satisfies this goal).
"""
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
include("utils.jl")

# Convenience constructors
"""
    Specification(problem::Problem)

Constructs a `Specification` of the appropriate concrete type based on the
information contained in the PDDL `Problem`. If no metric formula is specified,
a [`MinStepsGoal`](@ref) specification is returned. Otherwise, one of
[`MinMetricGoal`](@ref) or [`MaxMetricGoal`](@ref) are returned.
"""
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

"""
    Specification(goals::AbstractVector{<:Term})
    Specification(goal::Term)

Constructs a [`MinStepsGoal`](@ref) specification from one or more goals.
"""
Specification(goals::AbstractVector{<:Term}) = MinStepsGoal(goals)
Specification(goal::Term) = MinStepsGoal(goal)
