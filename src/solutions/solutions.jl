export Solution, NullSolution, OrderedSolution, PolicySolution
export get_action, best_action, rand_action

"Abstract solution type."
abstract type Solution end

"Return action for step `t` at `state`."
get_action(sol::Solution, t::Int, state::State) = error("Not implemented.")

"Null solution that indicates no plan was found."
struct NullSolution <: Solution
    status::Symbol
end

NullSolution() = NullSolution(:failure)

## Ordered solutions ##

"Abstract type for ordered planner solutions."
abstract type OrderedSolution <: Solution end

"Return action for step `t`."
get_action(sol::OrderedSolution, t::Int) = error("Not implemented.")
get_action(sol::OrderedSolution, t::Int, ::State) = get_action(sol, t)

# Ordered solutions should support indexing and iteration over action terms
Base.iterate(::OrderedSolution) = error("Not implemented.")
Base.iterate(::OrderedSolution, iterstate) = error("Not implemented.")
Base.getindex(::OrderedSolution, ::Int) = error("Not implemented.")
Base.eltype(::Type{OrderedSolution}) = Term

include("ordered_plan.jl")

## Policy-based solutions ##

"Abstract type for policy-based solutions."
abstract type PolicySolution <: Solution end

"Return action for the given state."
get_action(sol::PolicySolution, state::State) = best_action(sol, state)
get_action(sol::PolicySolution, ::Int, state::State) = get_action(sol, state)

"Returns the best action for the given state."
best_action(::PolicySolution, ::State) = error("Not implemented.")

"Samples an action according to the policy for the given state."
rand_action(::PolicySolution, ::State) = error("Not implemented.")

"Return value (i.e. expected future reward) of the given state (and action)."
get_value(::PolicySolution, ::State) = error("Not implemented.")
get_value(::PolicySolution, ::State, ::Term) = error("Not implemented.")

"Return Q-values of actions for the given state as an iterator over pairs."
get_action_values(::PolicySolution, ::State) = error("Not implemented.")

include("policy_value.jl")
include("random_policy.jl")
include("boltzmann_policy.jl")
include("epsilon_greedy.jl")
