export MultiSolution

"""
    MultiSolution(solutions::Solution...)
    MultiSolution(solutions::Tuple, [selector])

A combination of multiple [`Solution`](@ref)s, which are selected between
according to a `selector` function `(solutions, [state]) -> sol` that returns
the solution to use (which may depend on the current `state`). The `selector`
default to always returning the first solution.
"""
@auto_hash_equals struct MultiSolution{Ts <: Tuple, S} <: Solution
    solutions::Ts
    selector::S
end

function MultiSolution{Ts}(solutions::Ts) where {Ts <: Tuple}
    select_first(solutions, args...) = first(solutions)
    return MultiSolution(solutions, select_first)
end

MultiSolution(solutions::Ts) where {Ts <: Tuple} = MultiSolution{Ts}(solutions)
MultiSolution(solutions::Solution...) = MultiSolution(solutions)

function Base.copy(sol::MultiSolution)
    return MultiSolution(copy.(sol.solutions), sol.selector)
end

get_action(sol::MultiSolution, t::Int, state::State) =
    get_action(sol.selector(sol.solutions, state), t, state)

get_action(sol::MultiSolution, state::State) =
    get_action(sol.selector(sol.solutions, state), state)

rand_action(sol::MultiSolution, state::State) =
    rand_action(sol.selector(sol.solutions, state), state)

best_action(sol::MultiSolution, state::State) =
    best_action(sol.selector(sol.solutions, state), state)

has_values(sol::MultiSolution) =
    has_values(sol.selector(sol.solutions))

get_value(sol::MultiSolution, state::State) =
    get_value(sol.selector(sol.solutions, state), state)

get_value(sol::MultiSolution, state::State, action::Term) =
    get_value(sol.selector(sol.solutions, state), state, action)

get_action_values(sol::MultiSolution, state::State) =
    get_action_values(sol.selector(sol.solutions, state), state)

has_cached_value(sol::MultiSolution, state::State) =
    has_cached_value(sol.selector(sol.solutions, state), state)

has_cached_value(sol::MultiSolution, state::State, action::Term) =
    has_cached_value(sol.selector(sol.solutions, state), state, action)

has_cached_action_values(sol::MultiSolution, state::State) =
    has_cached_action_values(sol.selector(sol.solutions, state), state)

get_action_probs(sol::MultiSolution, state::State) =
    get_action_probs(sol.selector(sol.solutions, state), state)

get_action_prob(sol::MultiSolution, state::State, action::Term) =
    get_action_prob(sol.selector(sol.solutions, state), state, action)
