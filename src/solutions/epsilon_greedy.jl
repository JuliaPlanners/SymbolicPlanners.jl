export EpsilonGreedyPolicy

"""
    EpsilonGreedyPolicy(domain, policy, epsilon)
    EpsilonGreedyPolicy(domain, policy, epsilon, rng::AbstractRNG)

Policy that acts uniformly at random with `epsilon` chance, but otherwise 
selects the best action according the underlying `policy`. The `domain` has
to be provided to determine the actions available in each state.
"""
@auto_hash_equals struct EpsilonGreedyPolicy{
    P <: PolicySolution, R <: AbstractRNG
} <: PolicySolution
    domain::Domain
    policy::P
    epsilon::Float64
    rng::R
end

EpsilonGreedyPolicy(domain, policy, epsilon) =
    EpsilonGreedyPolicy(domain, policy, epsilon, Random.GLOBAL_RNG)

Base.copy(sol::EpsilonGreedyPolicy) =
    EpsilonGreedyPolicy(sol.domain, copy(sol.policy), sol.epsilon, sol.rng)

get_action(sol::EpsilonGreedyPolicy, state::State) =
    rand_action(sol, state)
best_action(sol::EpsilonGreedyPolicy, state::State) =
    best_action(sol.policy, state)
get_value(sol::EpsilonGreedyPolicy, state::State) =
    get_value(sol.policy, state)
get_value(sol::EpsilonGreedyPolicy, state::State, action::Term) =
    get_value(sol.policy, state, action)
get_action_values(sol::EpsilonGreedyPolicy, state::State) =
    get_action_values(sol.policy, state)

function rand_action(sol::EpsilonGreedyPolicy, state::State)
    if rand(sol.rng) < sol.epsilon
        actions = lazy_collect(available(sol.domain, state))
        return rand(sol.rng, actions)    
    else
        return best_action(sol.policy, state)
    end
end

function get_action_probs(sol::EpsilonGreedyPolicy, state::State)
    probs = Dict(act => sol.epsilon for act in available(sol.domain, state))
    n_actions = length(probs)
    if n_actions == 0 return probs end
    map!(x -> x / n_actions, values(probs))
    best_act = best_action(sol.policy, state)
    probs[best_act] += 1 - sol.epsilon
    return probs
end

function get_action_prob(sol::EpsilonGreedyPolicy, state::State, action::Term)
    n_actions = length(lazy_collect(available(sol.domain, state)))
    if n_actions == 0 return 0.0 end
    prob = sol.epsilon / n_actions
    best_act = best_action(sol.policy, state)
    if action == best_act
        return prob + (1 - sol.epsilon)
    else
        return prob
    end
end
