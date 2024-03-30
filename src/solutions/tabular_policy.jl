export TabularPolicy, TabularVPolicy

"""
    has_cached_value(sol, state)
    has_cached_value(sol, state, action)

Returns true if the value of `state` (and `action`) is cached in the state
value or action value table of `sol`.
"""
has_cached_value(sol::Solution, state::State) = false
has_cached_value(sol::Solution, state::State, action::Term) = false

"""
    TabularPolicy(V::Dict, Q::Dict, default)
    TabularPolicy(default = NullPolicy())

Policy solution where state values and action Q-values are stored in lookup
tables `V` and `Q`, where `V` maps state hashes to values, and `Q` maps state
hashes to dictionaries of Q-values for each action in the corresponding state.

A `default` policy can be specified, so that if a state doesn't already exist
in the lookup tables, the value returned by `default` will be used instead.
"""
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

function best_action(sol::TabularPolicy, state::State)
    q_values = get_action_values(sol, state)
    return isempty(q_values) ? missing : argmax(q_values)
end

function get_value(sol::TabularPolicy, state::State)
    return get(sol.V, hash(state)) do
        get_value(sol.default, state) |> Float64
    end
end

function get_value(sol::TabularPolicy, state_id::UInt, default=nothing)
    return get(sol.V, state_id, nothing)
end

function get_value(sol::TabularPolicy, state::State, action::Term)
    qs = get(sol.Q, hash(state), nothing)
    if qs === nothing
        return get_value(sol.default, state, action) |> Float64
    else
        return get(qs, action) do
            get_value(sol.default, state, action) |> Float64
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

function set_value!(sol::TabularPolicy, state::State, val::Real)
    set_value!(sol, hash(state), val)
end

function set_value!(sol::TabularPolicy, state_id::UInt, val::Real)
    sol.V[state_id] = val
end

function set_value!(sol::TabularPolicy, state::State, action::Term, val::Real)
    set_value!(sol, hash(state), action, val)
end

function set_value!(sol::TabularPolicy, state_id::UInt, action::Term, val::Real)
    qs = get!(sol.Q, state_id) do
        Dict{Term,Float64}()
    end
    qs[action] = val
end

function has_cached_value(sol::TabularPolicy, state::State)
    return has_cached_value(sol, hash(state))
end

function has_cached_value(sol::TabularPolicy, state_id::UInt)
    return haskey(sol.V, state_id)
end

function has_cached_value(sol::TabularPolicy, state::State, action::Term)
    return has_cached_value(sol, hash(state), action)
end

function has_cached_value(sol::TabularPolicy, state_id::UInt, action::Term)
    qs = get(sol.Q, state_id, nothing)
    return !isnothing(qs) && haskey(qs, action)
end

"""
    TabularVPolicy(V::Dict, domain, spec, default)
    TabularVPolicy(domain, spec, default = NullPolicy())

Policy solution where state values are stored in a lookup table `V` that maps
state hashes to values. The domain and specification also have to be provided,
so that the policy knows how to derive action Q-values in each state.

A `default` policy can be specified, so that if a state doesn't already exist
in the lookup table, the value returned by `default` will be used instead.
"""
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
        get_value(sol.default, state) |> Float64
    end
end

function get_value(sol::TabularVPolicy, state_id::UInt, default=nothing)
    return get(sol.V, state_id, default)
end

function get_value(sol::TabularVPolicy, state::State, action::Term)
    next_state = transition(sol.domain, state, action)
    if (has_action_goal(sol.spec) &&
        is_goal(sol.spec, sol.domain, next_state, action))
        next_v = 0.0
    else
        next_v = get_value(sol, next_state) |> Float64
    end
    r = get_reward(sol.spec, sol.domain, state, action, next_state)
    return get_discount(sol.spec) * next_v + r
end

get_action_values(sol::TabularVPolicy, state::State) =
    Dict(act => get_value(sol, state, act) for act in available(sol.domain, state))

function set_value!(sol::TabularVPolicy, state::State, val::Real)
    sol.V[hash(state)] = val
end

function set_value!(sol::TabularVPolicy, state_id::UInt, val::Real)
    sol.V[state_id] = val
end

function has_cached_value(sol::TabularVPolicy, state::State)
    return has_cached_value(sol, hash(state))
end

function has_cached_value(sol::TabularVPolicy, state_id::UInt)
    return haskey(sol.V, state_id)
end

function has_cached_value(sol::TabularVPolicy, state_id::UInt, action::Term)
    return false
end
