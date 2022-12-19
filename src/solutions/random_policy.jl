export RandomPolicy

"Policy that selects actions uniformly at random."
struct RandomPolicy{D <: Domain, R <: AbstractRNG} <: PolicySolution
    domain::D
    rng::R
end

RandomPolicy(domain) = RandomPolicy(domain, Random.GLOBAL_RNG)

get_action(sol::RandomPolicy, state::State) =
    rand_action(sol, state)

function rand_action(sol::RandomPolicy, state::State)
    actions = available(sol.domain, state)
    if Base.IteratorSize(actions) == Base.SizeUnknown()
        actions = collect(actions)
    end
    return rand(sol.rng, actions)
end

function get_action_probs(sol::RandomPolicy, state::State)
    probs = Dict(act => 1.0 for act in available(sol.domain, state))
    n_actions = length(probs)
    map!(x -> x / n_actions, values(probs))
    return probs
end