export FunctionalVPolicy, HeuristicVPolicy

"""
    FunctionalVPolicy(evaluator, domain, spec)

Policy solution where state values are defined by an `evaluator`, a one-argument
function that outputs a value estimate for each `state`. The domain and
specification also have to be provided, so that the policy knows how to derive
action Q-values for each state.
"""
@auto_hash_equals struct FunctionalVPolicy{
    F, D <: Domain, S <: Specification
} <: PolicySolution
    evaluator::F
    domain::D
    spec::S
end

Base.copy(sol::FunctionalVPolicy) =
    FunctionalVPolicy(sol.evaluator, sol.domain, sol.spec)

get_action(sol::FunctionalVPolicy, state::State) =
    best_action(sol, state)
rand_action(sol::FunctionalVPolicy, state::State) =
    best_action(sol, state)

function best_action(sol::FunctionalVPolicy, state::State)
    best_val = -Inf
    best_act = missing
    for act in available(sol.domain, state)
        val = get_value(sol, state, act)
        if val > best_val
            best_val = val
            best_act = act
        end 
    end
    return best_act
end

has_values(sol::FunctionalVPolicy) = true

get_value(sol::FunctionalVPolicy, state::State) =
    sol.evaluator(state)
get_action_values(sol::FunctionalVPolicy, state::State) =
    Dict(act => get_value(sol, state, act) for act in available(sol.domain, state))

function get_value(sol::FunctionalVPolicy, state::State, action::Term)
    next_state = transition(sol.domain, state, action)
    next_v = sol.evaluator(next_state)
    r = get_reward(sol.spec, sol.domain, state, action, next_state)
    return get_discount(sol.spec) * next_v + r
end

"""
    HeuristicVPolicy(heuristic:Heuristic, domain, spec)

Policy solution where state values are defined by the (negated) goal distance
estimates computed by a [`Heuristic`](@ref) for a `domain` and goal `spec`.
"""
@auto_hash_equals struct HeuristicVPolicy{
    H <: Heuristic, D <: Domain, S <: Specification
} <: PolicySolution
    heuristic::H
    domain::D
    spec::S
end

Base.copy(sol::HeuristicVPolicy) =
    HeuristicVPolicy(sol.heuristic, sol.domain, sol.spec)

get_action(sol::HeuristicVPolicy, state::State) =
    best_action(sol, state)
rand_action(sol::HeuristicVPolicy, state::State) =
    best_action(sol, state)

function best_action(sol::HeuristicVPolicy, state::State)
    best_val = -Inf
    best_act = missing
    for act in available(sol.domain, state)
        val = get_value(sol, state, act)
        if val > best_val
            best_val = val
            best_act = act
        end 
    end
    return best_act
end

has_values(sol::HeuristicVPolicy) = true

get_value(sol::HeuristicVPolicy, state::State) =
    -compute(sol.heuristic, sol.domain, state, sol.spec)
get_action_values(sol::HeuristicVPolicy, state::State) =
    Dict(act => get_value(sol, state, act) for act in available(sol.domain, state))

function get_value(sol::HeuristicVPolicy, state::State, action::Term)
    next_state = transition(sol.domain, state, action)
    next_v = get_value(sol, next_state)
    r = get_reward(sol.spec, sol.domain, state, action, next_state)
    return get_discount(sol.spec) * next_v + r
end
