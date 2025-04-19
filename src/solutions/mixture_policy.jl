export MixturePolicy

"""
    MixturePolicy(policies, [weights, rng::AbstractRNG])

A mixture of underlying `policies` with associated `weights`. If provided,
`weights` must be non-negative and sum to one. Otherwise a uniform mixture is
assumed.
"""
@auto_hash_equals struct MixturePolicy{Ps, R <: AbstractRNG} <: PolicySolution
    policies::Ps
    weights::Vector{Float64}
    rng::R
    function MixturePolicy{Ps, R}(
        policies::Ps, weights, rng::R
    ) where {Ps, R <: AbstractRNG}
        @assert length(policies) == length(weights)
        @assert all(w >= 0 for w in weights)
        @assert isapprox(sum(weights), 1.0)
        weights = convert(Vector{Float64}, weights)
        return new(policies, weights, rng)
    end
end

function MixturePolicy(
    policies::Vector{P},
    weights = ones(length(policies)) ./ length(policies),
    rng::R = Random.GLOBAL_RNG
) where {P, R <: AbstractRNG}
    return MixturePolicy{Vector{P}, R}(policies, weights, rng)
end

function MixturePolicy(
    policies::Ps,
    weights = ones(length(policies)) ./ length(policies),
    rng::R = Random.GLOBAL_RNG
) where {Ps <: Tuple, R <: AbstractRNG}
    return MixturePolicy{Ps, R}(policies, weights, rng)
end

function MixturePolicy(policies, rng::AbstractRNG)
    weights = ones(length(policies)) ./ length(policies)
    return MixturePolicy(policies, weights, rng)
end

function Base.show(io::IO, ::MIME"text/plain", sol::MixturePolicy)
    indent = get(io, :indent, "")
    show_struct(io, sol; indent = indent, show_fields_compact=(:weights,))
end

Base.copy(sol::MixturePolicy) =
    MixturePolicy(copy.(sol.policies), copy(sol.weights), sol.rng)

get_action(sol::MixturePolicy, state::State) =
    rand_action(sol, state)

function rand_action(sol::MixturePolicy, state::State)
    idx = sample(sol.rng, Weights(sol.weights))
    policy = sol.policies[idx]
    return rand_action(policy, state)
end

function get_action_probs(sol::MixturePolicy, state::State)
    probs = Dict{Term, Float64}()
    for (policy, weight) in zip(sol.policies, sol.weights)
        for (action, prob) in get_action_probs(policy, state)
            probs[action] = get(probs, action, 0.0) + prob * weight
        end
    end
    return probs
end

function get_action_prob(sol::MixturePolicy, state::State, action::Term)
    prob = 0.0
    for (policy, weight) in zip(sol.policies, sol.weights)
        prob += get_action_prob(policy, state, action) * weight
    end
    return prob
end

"""
    get_mixture_weights(sol)

Returns the mixture weights for a mixture policy.

    get_mixture_weights(sol, state, action; normalize = true)

Returns the posterior mixture weights for a mixture policy after an `action`
has been taken at `state`. Posterior weights are normalized by default, but 
this can be disabled by setting `normalize = false`.
"""
function get_mixture_weights end

function get_mixture_weights(sol::MixturePolicy)
    return sol.weights
end

function get_mixture_weights(sol::MixturePolicy, state::State, action::Term;
                             normalize::Bool = true)
    joint_probs = map(zip(sol.policies, sol.weights)) do (policy, weight)
        return get_action_prob(policy, state, action) * weight
    end
    new_weights = normalize ? joint_probs ./ sum(joint_probs) : joint_probs
    return new_weights
end
