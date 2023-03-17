export BoltzmannPolicy

"""
    BoltzmannPolicy(policy, temperature)
    BoltzmannPolicy(policy, temperature, rng::AbstractRNG)

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

unzip_pairs(ps) = unzip_pairs(collect(ps))
unzip_pairs(ps::AbstractDict) = collect(keys(ps)), collect(values(ps))
unzip_pairs(ps::AbstractArray{<:Pair}) = first.(ps), last.(ps)

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
