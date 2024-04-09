export EpsilonGreedyPolicy, EpsilonMixturePolicy

"""
    EpsilonGreedyPolicy(domain, policy, epsilon, [rng::AbstractRNG])

Policy that acts uniformly at random with `epsilon` chance, but otherwise 
selects the best action(s) according the underlying `policy`. If there is
more than one best action, tie-breaking occurs randomly. The `domain` has
to be provided to determine the actions available in each state.
"""
@auto_hash_equals struct EpsilonGreedyPolicy{
    D <: Domain, P, R <: AbstractRNG
} <: PolicySolution
    domain::D
    policy::P
    epsilon::Float64
    rng::R
end

EpsilonGreedyPolicy(domain, policy, epsilon) =
    EpsilonGreedyPolicy(domain, policy, epsilon, Random.GLOBAL_RNG)

function Base.show(io::IO, ::MIME"text/plain", sol::EpsilonGreedyPolicy)
    indent = get(io, :indent, "")
    show_struct(io, sol; indent = indent, show_fields=(:policy,))
end

Base.copy(sol::EpsilonGreedyPolicy) =
    EpsilonGreedyPolicy(sol.domain, copy(sol.policy), sol.epsilon, sol.rng)

get_action(sol::EpsilonGreedyPolicy, state::State) =
    rand_action(sol, state)
best_action(sol::EpsilonGreedyPolicy, state::State) =
    best_action(sol.policy, state)
has_values(sol::EpsilonGreedyPolicy) =
    has_values(sol.policy)
get_value(sol::EpsilonGreedyPolicy, state::State) =
    get_value(sol.policy, state)
get_value(sol::EpsilonGreedyPolicy, state::State, action::Term) =
    get_value(sol.policy, state, action)
get_action_values(sol::EpsilonGreedyPolicy, state::State) =
    get_action_values(sol.policy, state)
has_cached_value(sol::EpsilonGreedyPolicy, state::State) =
    has_cached_value(sol.policy, state)
has_cached_value(sol::EpsilonGreedyPolicy, state::State, action::Term) =
    has_cached_value(sol.policy, state, action)
has_cached_action_values(sol::EpsilonGreedyPolicy, state::State) =
    has_cached_action_values(sol.policy, state)

function rand_action(sol::EpsilonGreedyPolicy, state::State)
    if rand(sol.rng) < sol.epsilon
        # Sample an action at random
        actions = lazy_collect(available(sol.domain, state))
        return rand(sol.rng, actions)    
    elseif has_values(sol.policy)
        # Sample one of the highest value actions at random
        qs = get_action_values(sol.policy, state)
        if isempty(qs) return missing end
        q_max = maximum(values(qs))
        n_max = 0
        act = missing
        for (a, q) in qs # Reservoir sampling among maximal elements
            q < q_max && continue
            n_max += 1
            j = rand(sol.rng, 1:n_max)
            j == 1 && (act = a)
        end
        return act
    else
        # If policy does not store values, return the best action
        return best_action(sol.policy, state)
    end
end

function get_action_probs(sol::EpsilonGreedyPolicy, state::State)
    if has_values(sol.policy)
        qs = get_action_values(sol.policy, state)
        if isempty(qs) return Dict{Term,Float64}() end
        probs = Dict{Term,Float64}(act => sol.epsilon for act in keys(qs))
        n_actions = length(probs)
        map!(x -> x / n_actions, values(probs))
        q_max = maximum(values(qs))
        n_max = sum(q >= q_max for q in values(qs))
        for (act, q) in qs
            q < q_max && continue
            probs[act] = (1 - sol.epsilon) / n_max + get(probs, act, 0.0)
        end            
    else
        probs = Dict{Term,Float64}(act => sol.epsilon for
                                   act in available(sol.domain, state))
        n_actions = length(probs)
        if n_actions == 0 return probs end
        map!(x -> x / n_actions, values(probs))
        best_act = best_action(sol.policy, state)
        probs[best_act] = 1 - sol.epsilon + get(probs, best_act, 0.0)
    end
    return probs
end

function get_action_prob(sol::EpsilonGreedyPolicy, state::State, action::Term)
    if has_values(sol.policy)
        qs = get_action_values(sol.policy, state)
        if !haskey(qs, action) return 0.0 end
        q_max = maximum(values(qs))
        q_act = get(qs, action, -Inf)
        rand_prob = sol.epsilon / length(qs)
        if q_act >= q_max
            n_max = sum(q >= q_max for q in values(qs))
            return rand_prob + (1 - sol.epsilon) / n_max
        else
            return rand_prob
        end
    else
        actions = lazy_collect(available(sol.domain, state))
        n_actions = length(actions)
        if n_actions == 0 return 0.0 end
        rand_prob = sol.epsilon / n_actions
        best_act = best_action(sol.policy, state)
        if action == best_act
            return rand_prob + (1 - sol.epsilon)
        elseif action in actions
            return rand_prob
        else
            return 0.0
        end
    end
end

"""
    EpsilonMixturePolicy(domain, policy, epsilons, [weights, rng::AbstractRNG])

A mixture of epsilon-greedy policies with different `epsilons` and mixture 
`weights`, specified as `Vector`s. If provided, `weights` must be non-negative
and sum to one. Otherwise a uniform mixture is assumed. The `domain` is required
to determine the actions available in each state.
"""
@auto_hash_equals struct EpsilonMixturePolicy{
    D <: Domain, P, R <: AbstractRNG
} <: PolicySolution
    domain::D
    policy::P
    epsilons::Vector{Float64}
    weights::Vector{Float64}
    rng::R
    function EpsilonMixturePolicy{D, P, R}(
        domain::D, policy::P, epsilons, weights, rng::R
    ) where {D <: Domain, P, R <: AbstractRNG}
        @assert length(epsilons) == length(weights)
        @assert all(w >= 0 for w in weights)
        @assert isapprox(sum(weights), 1.0)
        epsilons = convert(Vector{Float64}, epsilons)
        weights = convert(Vector{Float64}, weights)
        return new(domain, policy, epsilons, weights, rng)
    end
end

function EpsilonMixturePolicy(
    domain::D,
    policy::P,
    epsilons,
    weights = ones(length(epsilons)) ./ length(epsilons),
    rng::R = Random.GLOBAL_RNG
) where {D <: Domain, P, R <: AbstractRNG}
    return EpsilonMixturePolicy{D, P, R}(domain, policy, epsilons, weights, rng)
end

function EpsilonMixturePolicy(domain, policy, epsilons, rng::AbstractRNG)
    weights = ones(length(epsilons)) ./ length(epsilons)
    return EpsilonMixturePolicy(domain, policy, epsilons, weights, rng)
end

function Base.show(io::IO, ::MIME"text/plain", sol::EpsilonMixturePolicy)
    indent = get(io, :indent, "")
    show_struct(io, sol; indent = indent, show_fields=(:policy,),
                show_fields_compact=(:epsilons, :weights))
end

Base.copy(sol::EpsilonMixturePolicy) =
    EpsilonMixturePolicy(sol.domain, copy(sol.policy), copy(sol.epsilons),
                         copy(sol.weights), sol.rng)

get_action(sol::EpsilonMixturePolicy, state::State) =
    rand_action(sol, state)
best_action(sol::EpsilonMixturePolicy, state::State) =
    best_action(sol.policy, state)
has_values(sol::EpsilonMixturePolicy) =
    has_values(sol.policy)
get_value(sol::EpsilonMixturePolicy, state::State) =
    get_value(sol.policy, state)
get_value(sol::EpsilonMixturePolicy, state::State, action::Term) =
    get_value(sol.policy, state, action)
get_action_values(sol::EpsilonMixturePolicy, state::State) =
    get_action_values(sol.policy, state)
has_cached_value(sol::EpsilonMixturePolicy, state::State) =
    has_cached_value(sol.policy, state)
has_cached_value(sol::EpsilonMixturePolicy, state::State, action::Term) =
    has_cached_value(sol.policy, state, action)
has_cached_action_values(sol::EpsilonMixturePolicy, state::State) =
    has_cached_action_values(sol.policy, state)

function rand_action(sol::EpsilonMixturePolicy, state::State)
    epsilon = sample(sol.rng, sol.epsilons, Weights(sol.weights))
    policy = EpsilonGreedyPolicy(sol.domain, sol.policy, epsilon, sol.rng)
    return rand_action(policy, state)
end

function get_action_probs(sol::EpsilonMixturePolicy, state::State)
    epsilon = (sol.epsilons' * sol.weights) # Marginal epsilon
    policy = EpsilonGreedyPolicy(sol.domain, sol.policy, epsilon, sol.rng)
    return get_action_probs(policy, state)
end

function get_action_prob(sol::EpsilonMixturePolicy, state::State, action::Term)
    epsilon = (sol.epsilons' * sol.weights) # Marginal epsilon
    policy = EpsilonGreedyPolicy(sol.domain, sol.policy, epsilon, sol.rng)
    return get_action_prob(policy, state, action)
end

function get_mixture_weights(sol::EpsilonMixturePolicy)
    return sol.weights
end

function get_mixture_weights(sol::EpsilonMixturePolicy,
                             state::State, action::Term)
    if has_values(sol.policy)
        qs = get_action_values(sol.policy, state)
        if !haskey(qs, action) return sol.weights end
        q_max = maximum(values(qs))
        n_actions = length(qs)
        n_max = sum(q >= q_max for q in values(qs))
        joint_probs = map(zip(sol.epsilons, sol.weights)) do (epsilon, weight)
            prob = epsilon * weight / n_actions
            prob += (qs[action] >= q_max) ? (1 - epsilon) * weight / n_max : 0.0
            return prob
        end
    else
        actions = lazy_collect(available(sol.domain, state))
        n_actions = length(actions)
        if (n_actions == 0 || !(action in actions)) return sol.weights end
        best_act = best_action(sol.policy, state)
        joint_probs = map(zip(sol.epsilons, sol.weights)) do (epsilon, weight)
            prob = epsilon * weight / n_actions
            prob += (action == best_act) ? (1 - epsilon) * weight : 0.0
            return prob
        end
    end
    new_weights = joint_probs ./ sum(joint_probs)
    return new_weights
end
