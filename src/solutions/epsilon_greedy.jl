export EpsilonGreedyPolicy

"Policy that acts uniformly at random with ϵ chance, but is otherwise greedy."
struct EpsilonGreedyPolicy{P <: PolicySolution, R <: AbstractRNG} <: PolicySolution
    domain::Domain
    policy::P
    epsilon::Float64
    rng::R
end

EpsilonGreedyPolicy(domain, policy, epsilon) =
    EpsilonGreedyPolicy(domain, policy, epsilon, Random.GLOBAL_RNG)

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
    best_act = best_action(sol.policy, state)
    actions = [available(state, sol.domain); best_act]
    probs = ones(length(actions)) * sol.epsilon
    probs[end] = 1 - sol.epsilon
    return sample(sol.rng, actions, Weights(probs, 1.0))
end
