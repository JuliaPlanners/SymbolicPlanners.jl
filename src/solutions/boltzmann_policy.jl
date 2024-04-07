export BoltzmannPolicy, BoltzmannMixturePolicy

"""
    BoltzmannPolicy(policy, temperature, [rng::AbstractRNG])

Policy that samples actions according to a Boltzmann distribution with the 
specified `temperature`. The unnormalized log probability of taking an action
``a`` in state ``s`` corresponds to its Q-value ``Q(s, a)`` divided by the
temperature ``T``:

```math
P(a|s) \\propto \\exp(Q(s, a) / T)
```

Higher temperatures lead to an increasingly random policy, whereas a temperature
of zero corresponds to a deterministic policy. Q-values are computed according
to the underlying `policy` provided as an argument to the constructor.

Note that wrapping an existing policy in a `BoltzmannPolicy` does not ensure
consistency of the state values ``V`` and Q-values ``Q`` according to the 
Bellman equation, since this would require repeated Bellman updates to ensure
convergence.
"""
@auto_hash_equals struct BoltzmannPolicy{P, R <: AbstractRNG} <: PolicySolution
    policy::P
    temperature::Float64
    rng::R
end

BoltzmannPolicy(policy::BoltzmannPolicy, temperature, rng) =
    BoltzmannPolicy(policy.policy, temperature, rng)

BoltzmannPolicy(policy, temperature) =
    BoltzmannPolicy(policy, temperature, Random.GLOBAL_RNG)

function Base.show(io::IO, ::MIME"text/plain", sol::BoltzmannPolicy)
    indent = get(io, :indent, "")
    show_struct(io, sol; indent = indent, show_fields=(:policy,))
end
    
Base.copy(sol::BoltzmannPolicy) =
    BoltzmannPolicy(copy(sol.policy), sol.temperature, sol.rng)

get_action(sol::BoltzmannPolicy, state::State) =
    rand_action(sol, state)
best_action(sol::BoltzmannPolicy, state::State) =
    best_action(sol.policy, state)
get_value(sol::BoltzmannPolicy, state::State) =
    get_value(sol.policy, state)
get_value(sol::BoltzmannPolicy, state::State, action::Term) =
    get_value(sol.policy, state, action)
get_action_values(sol::BoltzmannPolicy, state::State) =
    get_action_values(sol.policy, state)
has_cached_value(sol::BoltzmannPolicy, state::State) =
    has_cached_value(sol.policy, state)
has_cached_value(sol::BoltzmannPolicy, state::State, action::Term) =
    has_cached_value(sol.policy, state, action)

function rand_action(sol::BoltzmannPolicy, state::State)
    if sol.temperature == 0
        return best_action(sol, state)
    end
    chosen_act, chosen_score = missing, -Inf
    for (act, q) in get_action_values(sol, state)
        score = q / sol.temperature + randgumbel()
        if score > chosen_score
            chosen_act = act
            chosen_score = score
        end
    end
    return chosen_act
end

function get_action_probs(sol::BoltzmannPolicy, state::State)
    action_values = get_action_values(sol, state)
    if isempty(action_values)
        return Dict{Term,Float64}()
    end
    actions, q_values = unzip_pairs(action_values)
    if sol.temperature == 0
        probs = zeros(length(actions))
        probs[argmax(q_values)] = 1.0
    else
        probs = softmax(q_values ./ sol.temperature)
    end 
    probs = Dict(zip(actions, probs))
    return probs
end

function get_action_prob(sol::BoltzmannPolicy, state::State, action::Term)
    if sol.temperature == 0.0
        return action == best_action(sol, state) ? 1.0 : 0.0
    end
    action_values = get_action_values(sol, state)
    if isempty(action_values)
        return 0.0
    end
    actions, q_values = unzip_pairs(action_values)
    probs = softmax(q_values ./ sol.temperature)
    for (a, p) in zip(actions, probs)
        a == action && return p
    end
    return 0.0
end

"""
    BoltzmannMixturePolicy(policy, temperatures, [weights, rng::AbstractRNG])

A mixture of Boltzmann policies with different `temperatures` and mixture 
`weights`, specified as `Vector`s. If provided, `weights` must be non-negative
and sum to one. Otherwise a uniform mixture is assumed. Q-values are computed
according to the underlying `policy` provided as an argument to the constructor.
"""
@auto_hash_equals struct BoltzmannMixturePolicy{P, R <: AbstractRNG} <: PolicySolution
    policy::P
    temperatures::Vector{Float64}
    weights::Vector{Float64}
    rng::R
    function BoltzmannMixturePolicy{P, R}(
        policy::P, temperatures, weights, rng::R
    ) where {P, R <: AbstractRNG}
        @assert length(temperatures) == length(weights)
        @assert all(w >= 0 for w in weights)
        @assert isapprox(sum(weights), 1.0)
        temperatures = convert(Vector{Float64}, temperatures)
        weights = convert(Vector{Float64}, weights)
        return new(policy, temperatures, weights, rng)
    end
end

function BoltzmannMixturePolicy(
    policy::P,
    temperatures,
    weights = ones(length(temperatures)) ./ length(temperatures),
    rng::R = Random.GLOBAL_RNG
) where {P, R <: AbstractRNG}
    return BoltzmannMixturePolicy{P, R}(policy, temperatures, weights, rng)
end

function BoltzmannMixturePolicy(policy, temperatures, rng::AbstractRNG)
    weights = ones(length(temperatures)) ./ length(temperatures)
    return BoltzmannMixturePolicy(policy, temperatures, weights, rng)
end

function Base.show(io::IO, ::MIME"text/plain", sol::BoltzmannMixturePolicy)
    indent = get(io, :indent, "")
    show_struct(io, sol; indent = indent, show_fields=(:policy,),
                show_fields_compact=(:temperatures, :weights))
end

Base.copy(sol::BoltzmannMixturePolicy) =
    BoltzmannMixturePolicy(copy(sol.policy), sol.temperatures,
                           sol.weights, sol.rng)

get_action(sol::BoltzmannMixturePolicy, state::State) =
    rand_action(sol, state)
best_action(sol::BoltzmannMixturePolicy, state::State) =
    best_action(sol.policy, state)
get_value(sol::BoltzmannMixturePolicy, state::State) =
    get_value(sol.policy, state)
get_value(sol::BoltzmannMixturePolicy, state::State, action::Term) =
    get_value(sol.policy, state, action)
get_action_values(sol::BoltzmannMixturePolicy, state::State) =
    get_action_values(sol.policy, state)
has_cached_value(sol::BoltzmannMixturePolicy, state::State) =
    has_cached_value(sol.policy, state)
has_cached_value(sol::BoltzmannMixturePolicy, state::State, action::Term) =
    has_cached_value(sol.policy, state, action)

function rand_action(sol::BoltzmannMixturePolicy, state::State)
    temperature = sample(sol.rng, sol.temperatures, Weights(sol.weights))
    if temperature == 0
        return best_action(sol, state)
    end
    chosen_act, chosen_score = missing, -Inf
    for (act, q) in get_action_values(sol, state)
        score = q / temperature + randgumbel()
        if score > chosen_score
            chosen_act = act
            chosen_score = score
        end
    end
    return chosen_act
end

function get_action_probs(sol::BoltzmannMixturePolicy, state::State)
    action_values = get_action_values(sol, state)
    if isempty(action_values)
        return Dict{Term,Float64}()
    end
    actions, q_values = unzip_pairs(action_values)
    probs = zeros(length(actions))
    for (temp, weight) in zip(sol.temperatures, sol.weights)
        if temp == 0
            probs[argmax(q_values)] += weight
        else
            probs .+= softmax(q_values ./ temp) .* weight
        end
    end
    probs = Dict(zip(actions, probs))
    return probs
end

function get_action_prob(sol::BoltzmannMixturePolicy,
                         state::State, action::Term)
    action_values = get_action_values(sol, state)
    if isempty(action_values) || !haskey(action_values, action)
        return 0.0
    end
    actions, q_values = unzip_pairs(action_values)
    act_idx = findfirst(==(action), actions)
    act_prob = 0.0
    for (temp, weight) in zip(sol.temperatures, sol.weights)
        if temp == 0
            act_prob += (action == actions[argmax(q_values)]) * weight
        else
            probs = softmax(q_values ./ temp)
            act_prob += probs[act_idx] * weight
        end
    end
    return act_prob
end

function get_mixture_weights(sol::BoltzmannMixturePolicy)
    return sol.weights
end

function get_mixture_weights(sol::BoltzmannMixturePolicy,
                             state::State, action::Term)
    action_values = get_action_values(sol, state)
    if isempty(action_values) || !haskey(action_values, action)
        return sol.weights
    end
    actions, q_values = unzip_pairs(action_values)
    act_idx = findfirst(==(action), actions)
    joint_probs = zeros(length(sol.weights))
    joint_probs = map(zip(sol.temperatures, sol.weights)) do (temp, weight)
        if temp == 0
            return act_idx == argmax(q_values) ? weight : 0.0
        else
            probs = softmax(q_values ./ temp)
            return probs[act_idx] * weight
        end
    end
    new_weights = joint_probs ./ sum(joint_probs)
    return new_weights
end
