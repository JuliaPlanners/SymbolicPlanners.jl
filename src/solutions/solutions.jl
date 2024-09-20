export Solution, NullSolution, OrderedSolution, PolicySolution, NullPolicy
export get_action, best_action, rand_action

"""
$(TYPEDEF)

Abstract type for solutions to planning problems. Minimally, a `Solution`
should define what action should be taken at a particular step `t`, or at a 
particular `state`, by implementing [`get_action`](@ref).
"""
abstract type Solution end

function Base.show(io::IO, ::MIME"text/plain", sol::Solution)
    indent = get(io, :indent, "")
    show_struct(io, sol; indent = indent)
end

"""
$(SIGNATURES)

Return an action for step `t` at `state`.
"""
get_action(sol::Solution, t::Int, state::State) = error("Not implemented.")

"""
    NullSolution([status])

Null solution that indicates the problem was unsolved. The `status` field 
can be used to denote why the problem was unsolved. Defaults to `:failure`.
"""
@auto_hash_equals struct NullSolution <: Solution
    status::Symbol
end

NullSolution() = NullSolution(:failure)

Base.copy(sol::NullSolution) = NullSolution(sol.status)

## Ordered solutions ##

"""
$(TYPEDEF)

Abstract type for ordered planning solutions. `OrderedSolution`s should satisfy
the iteration interface. Calling `get_action(sol, t::Int)` on an ordered 
solution should return the intended action for step `t`.
"""
abstract type OrderedSolution <: Solution end

"""
$(SIGNATURES)

Return action for step `t`.
"""
get_action(sol::OrderedSolution, t::Int) = error("Not implemented.")
get_action(sol::OrderedSolution, t::Int, ::State) = get_action(sol, t)

# Ordered solutions should support indexing and iteration over action terms
Base.iterate(::OrderedSolution) = error("Not implemented.")
Base.iterate(::OrderedSolution, iterstate) = error("Not implemented.")
Base.getindex(::OrderedSolution, ::Int) = error("Not implemented.")
Base.length(sol::OrderedSolution) = error("Not implemented.")
Base.eltype(::Type{<:OrderedSolution}) = Term
Base.eltype(::T) where {T <: OrderedSolution} = eltype(T)

## Policy-based solutions ##

"""
$(TYPEDEF)

Abstract type for policy solutions. Minimally, `PolicySolution`s should
implement the `get_action(sol, state::State)` method, defining the (potentially
random) action to be taken at a particular `state`.
"""
abstract type PolicySolution <: Solution end

"""
$(SIGNATURES)

Return action for the given state. If no actions are available, return `missing`.
"""
get_action(sol::PolicySolution, state::State) = best_action(sol, state)
get_action(sol::PolicySolution, ::Int, state::State) = get_action(sol, state)

"""
$(SIGNATURES)

Returns the best action for the given state. If no actions are available,
return `missing`.
"""
best_action(sol::PolicySolution, state::State) = error("Not implemented.")

"""
$(SIGNATURES)

Samples an action according to the policy for the given state. If no actions are
available, return `missing`.
"""
rand_action(sol::PolicySolution, state::State) = error("Not implemented.")

"""
$(SIGNATURES)

Trait that denotes whether the solution stores a value function.
"""
has_values(sol::PolicySolution) = false
has_values(sol::Solution) = false

"""
    get_value(sol, state)
    get_value(sol, state, action)

Return value (i.e. expected future reward) of the given `state` (and `action`).
"""
get_value(::PolicySolution, ::State) = error("Not implemented.")
get_value(::PolicySolution, ::State, ::Term) = error("Not implemented.")

"""
$(SIGNATURES)

Return a dictionary of action Q-values for the given `state`.
"""
get_action_values(sol::PolicySolution, state::State) = error("Not implemented.")

"""
$(SIGNATURES)

Return a dictionary of action probabilities for the given `state`. If 
no actions are available, return an empty dictionary.
"""
function get_action_probs(sol::PolicySolution, state::State)
    best_act = best_action(sol, state)
    actions = keys(get_action_values(sol, state))
    probs = Dict{Term,Float64}(a => (a == best_act ? 1. : 0.) for a in actions)
    return probs
end

"""
$(SIGNATURES)

Return the probability of taking an `action` at the given `state`. If 
the `action` is not available, return zero.
"""
get_action_prob(sol::PolicySolution, state::State, action::Term) =
    action == best_action(sol, state) ? 1.0 : 0.0

"""
    NullPolicy()

Null policy which returns `missing` for calls to [`get_action`](@ref), etc.
"""
struct NullPolicy <: PolicySolution end

Base.copy(sol::NullPolicy) = sol

get_action(sol::NullPolicy, state::State) = missing
best_action(sol::NullPolicy, state::State) = missing
rand_action(sol::NullPolicy, state::State) = missing

get_action_probs(sol::NullPolicy, state::State) = Dict{Term,Float64}()
get_action_prob(sol::NullPolicy, state::State, action::Term) = 0.0

## Solution library ##

include("ordered_plan.jl")
include("path_search.jl")
include("random_policy.jl")
include("tabular_policy.jl")
include("functional_policy.jl")
include("boltzmann_policy.jl")
include("epsilon_greedy.jl")
include("mixture_policy.jl")
include("reusable_tree.jl")
include("multi_solution.jl")
