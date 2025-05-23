export BoltzmannPolicy, BoltzmannMixturePolicy

"""
    BoltzmannPolicy(policy, temperature, [epsilon, rng::AbstractRNG])

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

To prevent zero-probability actions, an `epsilon` chance of taking an action 
uniformly at random can be specified.

Note that wrapping an existing policy in a `BoltzmannPolicy` does not ensure
consistency of the state values ``V`` and Q-values ``Q`` according to the 
Bellman equation, since this would require repeated Bellman updates to ensure
convergence.
"""
@auto_hash_equals struct BoltzmannPolicy{P, R <: AbstractRNG} <: PolicySolution
    policy::P
    temperature::Float64
    epsilon::Float64
    rng::R
end

BoltzmannPolicy(policy::BoltzmannPolicy, temperature, epsilon, rng) =
    BoltzmannPolicy(policy.policy, temperature, epsilon, rng)
BoltzmannPolicy(policy, temperature, epsilon = 0.0) =
    BoltzmannPolicy(policy, temperature, epsilon, Random.GLOBAL_RNG)
BoltzmannPolicy(policy, temperature, rng::AbstractRNG) =
    BoltzmannPolicy(policy, temperature, 0.0, rng)

function Base.show(io::IO, ::MIME"text/plain", sol::BoltzmannPolicy)
    indent = get(io, :indent, "")
    show_struct(io, sol; indent = indent, show_fields=(:policy,))
end
    
Base.copy(sol::BoltzmannPolicy) =
    BoltzmannPolicy(copy(sol.policy), sol.temperature,
                    sol.epsilon, sol.rng)

get_action(sol::BoltzmannPolicy, state::State) =
    rand_action(sol, state)
best_action(sol::BoltzmannPolicy, state::State) =
    best_action(sol.policy, state)
has_values(sol::BoltzmannPolicy) =
    has_values(sol.policy)
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
has_cached_action_values(sol::BoltzmannPolicy, state::State) =
    has_cached_action_values(sol.policy, state)

function rand_action(sol::BoltzmannPolicy, state::State)
    if sol.epsilon > 0 && (rand(sol.rng) < sol.epsilon)
        # Sample an action at random
        return rand(sol.rng, keys(get_action_values(sol, state)))
    elseif sol.temperature == 0
        # Reservoir sampling among maximal elements
        qs = get_action_values(sol, state)
        if isempty(qs) return missing end
        q_max = maximum(values(qs))
        n_max = 0
        chosen_act = missing
        for (act, q) in qs 
            q < q_max && continue
            n_max += 1
            j = rand(sol.rng, 1:n_max)
            j == 1 && (chosen_act = act)
        end
        return chosen_act
    else
        # Reservoir sampling via Gumbel-max trick
        chosen_act, chosen_score = missing, -Inf
        for (act, q) in get_action_values(sol, state)
            score = q / sol.temperature + randgumbel(sol.rng)
            if score > chosen_score
                chosen_act = act
                chosen_score = score
            end
        end
        return chosen_act
    end
end

function get_action_probs(sol::BoltzmannPolicy, state::State)
    action_values = get_action_values(sol, state)
    if isempty(action_values)
        return Dict{Term,Float64}()
    end
    actions, q_values = unzip_pairs(action_values)
    if sol.temperature == 0
        q_max = maximum(q_values)
        n_max = sum(q >= q_max for q in q_values)
        probs = [q >= q_max ? 1.0 / n_max : 0.0 for q in q_values]
    else
        probs = softmax(q_values ./ sol.temperature)
    end
    sol.epsilon > 0 && add_epsilon_probs!(probs, sol.epsilon)
    probs = Dict(zip(actions, probs))
    return probs
end

function get_action_prob(sol::BoltzmannPolicy, state::State, action::Term)
    action_values = get_action_values(sol, state)
    if isempty(action_values)
        return 0.0
    end
    actions, q_values = unzip_pairs(action_values)
    if sol.temperature == 0.0
        q_max = maximum(q_values)
        n_max = sum(q >= q_max for q in q_values)
        q_act = get(action_values, action, -Inf)
        return q_act >= q_max ? 1.0 / n_max : 0.0
    else
        probs = softmax(q_values ./ sol.temperature)
        sol.epsilon > 0 && add_epsilon_probs!(probs, sol.epsilon)
        for (a, p) in zip(actions, probs)
            a == action && return p
        end
        return 0.0
    end
end

"""
    BoltzmannMixturePolicy(policy, temperatures, [weights,]
                           [epsilon, rng::AbstractRNG])

A mixture of Boltzmann policies with different `temperatures` and mixture 
`weights`, specified as `Vector`s. If provided, `weights` must be non-negative
and sum to one. Otherwise a uniform mixture is assumed. Q-values are computed
according to the underlying `policy` provided as an argument to the constructor.

Similar to the [`BoltzmannPolicy`](@ref), `epsilon` can be specified to add a
an `epsilon` chance of taking an action uniformly at random.
"""
@auto_hash_equals struct BoltzmannMixturePolicy{P, R <: AbstractRNG} <: PolicySolution
    policy::P
    temperatures::Vector{Float64}
    weights::Vector{Float64}
    epsilon::Float64
    rng::R
    function BoltzmannMixturePolicy{P, R}(
        policy::P, temperatures, weights, epsilon, rng::R
    ) where {P, R <: AbstractRNG}
        @assert length(temperatures) == length(weights)
        @assert all(w >= 0 for w in weights)
        @assert isapprox(sum(weights), 1.0)
        @assert epsilon >= 0
        temperatures = convert(Vector{Float64}, temperatures)
        weights = convert(Vector{Float64}, weights)
        return new(policy, temperatures, weights, epsilon, rng)
    end
end

function BoltzmannMixturePolicy(
    policy::P,
    temperatures,
    weights = ones(length(temperatures)) ./ length(temperatures),
    epsilon = 0.0,
    rng::R = Random.GLOBAL_RNG
) where {P, R <: AbstractRNG}
    return BoltzmannMixturePolicy{P, R}(
        policy, temperatures, weights, epsilon, rng
    )
end

function BoltzmannMixturePolicy(policy, temperatures, rng::AbstractRNG)
    weights = ones(length(temperatures)) ./ length(temperatures)
    return BoltzmannMixturePolicy(policy, temperatures, weights, 0.0, rng)
end

function Base.show(io::IO, ::MIME"text/plain", sol::BoltzmannMixturePolicy)
    indent = get(io, :indent, "")
    show_struct(io, sol; indent = indent, show_fields=(:policy,),
                show_fields_compact=(:temperatures, :weights))
end

Base.copy(sol::BoltzmannMixturePolicy) =
    BoltzmannMixturePolicy(copy(sol.policy), copy(sol.temperatures),
                           copy(sol.weights), copy(sol.epsilon), sol.rng)

get_action(sol::BoltzmannMixturePolicy, state::State) =
    rand_action(sol, state)
best_action(sol::BoltzmannMixturePolicy, state::State) =
    best_action(sol.policy, state)
has_values(sol::BoltzmannMixturePolicy) = 
    has_values(sol.policy)
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
has_cached_action_values(sol::BoltzmannMixturePolicy, state::State) =
    has_cached_action_values(sol.policy, state)

function rand_action(sol::BoltzmannMixturePolicy, state::State)
    temperature = sample(sol.rng, sol.temperatures, Weights(sol.weights))
    policy = BoltzmannPolicy(sol.policy, temperature, sol.epsilon, sol.rng)
    return rand_action(policy, state)
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
            q_max = maximum(q_values)
            n_max = sum(q >= q_max for q in q_values)
            for (i, q) in enumerate(q_values)
                q < q_max && continue
                probs[i] += weight / n_max
            end
        else
            probs .+= softmax(q_values ./ temp) .* weight
        end
    end
    sol.epsilon > 0 && add_epsilon_probs!(probs, sol.epsilon)
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
            q_max = maximum(q_values)
            n_max = sum(q >= q_max for q in q_values)
            q_act = q_values[act_idx]
            act_prob += q_act >= q_max ? weight / n_max : 0.0
        else
            probs = softmax(q_values ./ temp)
            act_prob += probs[act_idx] * weight
        end
    end
    if sol.epsilon > 0
        act_prob *= (1 - sol.epsilon)
        act_prob += sol.epsilon / length(actions)
    end
    return act_prob
end

function get_mixture_weights(sol::BoltzmannMixturePolicy)
    return sol.weights
end

function get_mixture_weights(sol::BoltzmannMixturePolicy,
                             state::State, action::Term;
                             normalize::Bool = true)
    action_values = get_action_values(sol, state)
    if isempty(action_values) || !haskey(action_values, action)
        return sol.weights
    end
    actions, q_values = unzip_pairs(action_values)
    act_idx = findfirst(==(action), actions)
    joint_probs = zeros(length(sol.weights))
    joint_probs = map(zip(sol.temperatures, sol.weights)) do (temp, weight)
        if temp == 0
            q_max = maximum(q_values)
            n_max = sum(q >= q_max for q in q_values)
            q_act = q_values[act_idx]
            act_prob = q_act >= q_max ? (1.0 / n_max) : 0.0
        else
            probs = softmax(q_values ./ temp)
            act_prob = probs[act_idx]
        end
        if sol.epsilon > 0
            act_prob *= (1 - sol.epsilon)
            act_prob += sol.epsilon / length(actions)
        end
        return act_prob * weight
    end
    new_weights = normalize ? joint_probs ./ sum(joint_probs) : joint_probs
    return new_weights
end
