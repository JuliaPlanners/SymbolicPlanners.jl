export RandomPolicy

"Policy that selects actions uniformly at random."
@auto_hash_equals struct RandomPolicy{
    D <: Domain, R <: AbstractRNG
} <: PolicySolution
    domain::D
    rng::R
end

RandomPolicy(domain) = RandomPolicy(domain, Random.GLOBAL_RNG)

Base.copy(sol::RandomPolicy) =
    RandomPolicy(sol.domain, sol.rng)

get_action(sol::RandomPolicy, state::State) =
    rand_action(sol, state)

function rand_action(sol::RandomPolicy, state::State)
    actions = lazy_collect(available(sol.domain, state))
    return rand(sol.rng, actions)
end

function get_action_probs(sol::RandomPolicy, state::State)
    probs = Dict(act => 1.0 for act in available(sol.domain, state))
    n_actions = length(probs)
    map!(x -> x / n_actions, values(probs))
    return probs
end

function get_action_prob(sol::RandomPolicy, state::State, action::Term)
    actions = lazy_collect(available(sol.domain, state))
    return action in actions ? (1.0 / length(actions)) : 0.0
end
