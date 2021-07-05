export RandomPolicy

"Policy that selects actions uniformly at random."
struct RandomPolicy{R <: AbstractRNG} <: PolicySolution
    domain::Domain
    rng::R
end

RandomPolicy(domain) = RandomPolicy(domain, Random.GLOBAL_RNG)

get_action(sol::RandomPolicy, state::State) =
    rand_action(sol, state)
rand_action(sol::RandomPolicy, state::State) =
    rand(sol.rng, available(state, sol.domain))
