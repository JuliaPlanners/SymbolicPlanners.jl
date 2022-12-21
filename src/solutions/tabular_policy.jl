export TabularPolicy, TabularVPolicy

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

"Policy solution where state values are stored in a lookup table."
@auto_hash_equals struct TabularVPolicy{
    D <: Domain,
    S <: Specification,
    P <: PolicySolution,
} <: PolicySolution
    V::Dict{UInt64,Float64} # Value table
    domain::D # Domain for determining actions and transitions
    spec::S # Specification that determines state-action rewards
    default::P # Default fallback policy
end

TabularVPolicy(domain::Domain, spec::Specification) =
    TabularVPolicy(domain, spec, NullPolicy())

TabularVPolicy(domain::Domain, spec::Specification, default::PolicySolution) =
    TabularVPolicy(Dict{UInt64,Float64}(), domain, spec, default)

function Base.copy(sol::TabularVPolicy)
    V = copy(sol.V)
    return TabularVPolicy(V, sol.domain, sol.spec, copy(sol.default))
end

get_action(sol::TabularVPolicy, state::State) =
    best_action(sol, state)
rand_action(sol::TabularVPolicy, state::State) =
    best_action(sol, state)

function best_action(sol::TabularVPolicy, state::State)
    best_val = -Inf
    best_act = missing
    for act in available(sol.domain, state)
        val = get_value(sol, state, act)
        if val > best_val
            best_val = val
            best_act = act
        end 
    end
    return best_act
end

function get_value(sol::TabularVPolicy, state::State)
    return get(sol.V, hash(state)) do
        get_value(sol.default, state)
    end
end

get_action_values(sol::TabularVPolicy, state::State) =
    Dict(act => get_value(sol, state, act) for act in available(sol.domain, state))

function get_value(sol::TabularVPolicy, state::State, action::Term)
    next_state = transition(sol.domain, state, action)
    next_v = get_value(sol, next_state)
    r = get_reward(sol.spec, sol.domain, state, action, next_state)
    return get_discount(sol.spec) * next_v + r
end
