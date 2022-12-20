export Solution, NullSolution, OrderedSolution, PolicySolution, NullPolicy
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
Base.length(sol::OrderedSolution) = error("Not implemented.")
Base.eltype(::Type{<:OrderedSolution}) = Term
Base.eltype(::T) where {T <: OrderedSolution} = eltype(T)

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

"Return Q-values of actions for the given state as a dictionary."
get_action_values(::PolicySolution, ::State) = error("Not implemented.")

"Return probability of each action for the given state as a dictionary."
function get_action_probs(sol::PolicySolution, state::State)
    best_act = best_action(sol, state)
    actions = keys(get_action_values(sol, state))
    probs = Dict(act => (act == best_act ? 1.0 : 0.0) for act in actions)
    return probs
end

"Null policy which has no method implementations."
struct NullPolicy <: PolicySolution end

include("random_policy.jl")
include("tabular_policy.jl")
include("functional_policy.jl")
include("boltzmann_policy.jl")
include("epsilon_greedy.jl")
