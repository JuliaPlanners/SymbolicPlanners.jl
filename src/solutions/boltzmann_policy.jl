export BoltzmannPolicy

"Boltzmann policy with softmax action noise."
@auto_hash_equals struct BoltzmannPolicy{
    P <: PolicySolution, R <: AbstractRNG
} <: PolicySolution
    policy::P
    temperature::Float64
    rng::R
end

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
    actions, q_values = unzip_pairs(get_action_values(sol, state))
    if sol.temperature == 0
        probs = zeros(length(actions))
        probs[argmax(q_values)] = 1.0
    else
        probs = softmax(q_values ./ sol.temperature)
    end
    return sample(sol.rng, actions, Weights(probs, 1.0))
end

function get_action_probs(sol::BoltzmannPolicy, state::State)
    actions, q_values = unzip_pairs(get_action_values(sol, state))
    if sol.temperature == 0
        probs = zeros(length(actions))
        probs[argmax(q_values)] = 1.0
    else
        probs = softmax(q_values ./ sol.temperature)
    end 
    probs = Dict(zip(actions, probs))
    return probs
end

"Convert vector of scores to probabiities."
function softmax(scores)
    if isempty(scores) return Float64[] end
    ws = exp.(scores .- maximum(scores))
    z = sum(ws)
    return isnan(z) ? ones(length(scores)) ./ length(scores) : ws ./ z
end
