export BoltzmannPolicy

"Boltzmann policy with softmax action noise."
@auto_hash_equals struct BoltzmannPolicy{
    P <: PolicySolution, R <: AbstractRNG
} <: PolicySolution
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
