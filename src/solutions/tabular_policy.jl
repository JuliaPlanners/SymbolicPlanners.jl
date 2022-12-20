export TabularPolicy

"Policy solution where values and Q-values are stored in a lookup table."
@auto_hash_equals struct TabularPolicy{P <: PolicySolution} <: PolicySolution
    V::Dict{UInt64,Float64} # Value table
    Q::Dict{UInt64,Dict{Term,Float64}} # Q-value table
    default::P # Default fallback policy
end

TabularPolicy() =
    TabularPolicy(Dict(), Dict(), NullPolicy())

TabularPolicy(default::P) where {P <: PolicySolution} =
    TabularPolicy{P}(Dict(), Dict(), default)

TabularPolicy(V, Q, policy::P) where {P <: PolicySolution} =
    TabularPolicy{P}(V, Q, policy)

function Base.copy(sol::TabularPolicy)
    V = copy(sol.V)
    Q = Dict(s => copy(qs) for (s, qs) in sol.Q)
    return TabularPolicy(V, Q, copy(sol.default))
end

get_action(sol::TabularPolicy, state::State) =
    best_action(sol, state)
rand_action(sol::TabularPolicy, state::State) =
    best_action(sol, state)
best_action(sol::TabularPolicy, state::State) =
    argmax(get_action_values(sol, state))

function get_value(sol::TabularPolicy, state::State)
    return get(sol.V, hash(state)) do
        get_value(sol.default, state)
    end
end

function get_value(sol::TabularPolicy, state::State, action::Term)
    qs = get(sol.Q, hash(state), nothing)
    if qs === nothing
        return get_value(sol.default, state, action)
    else
        return get(qs, action) do
            get_value(sol.default, state, action)
        end
    end
end

function get_action_values(sol::TabularPolicy, state::State)
    qs = get(sol.Q, hash(state), nothing)
    if qs === nothing
        return get_action_values(sol.default, state)
    else
        return qs
    end
end
