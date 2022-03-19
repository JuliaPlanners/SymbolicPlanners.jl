export RandomPolicy

"Policy that selects actions uniformly at random."
struct RandomPolicy{D <: Domain, R <: AbstractRNG} <: PolicySolution
    domain::D
    rng::R
end

RandomPolicy(domain) = RandomPolicy(domain, Random.GLOBAL_RNG)

get_action(sol::RandomPolicy, state::State) =
    rand_action(sol, state)
rand_action(sol::RandomPolicy, state::State) =
    rand(sol.rng, collect(available(sol.domain, state)))
