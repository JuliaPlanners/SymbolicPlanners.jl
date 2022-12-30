export FunctionalVPolicy, FunctionalQPolicy

"""
    FunctionalVPolicy(evaluator, domain, spec)
    FunctionalVPolicy(heuristic:Heuristic, domain, spec)

Policy solution where state values are defined by an `evaluator`, a one-argument
function that outputs a value estimate for each `state`. An `evaluator` can
be automatically constructed from a `heuristic`, by negating the heuristic's
estimate of the distance to the goal.
    
The domain and specification also have to be provided, so that the policy knows
how to derive action Q-values for each state.
"""
@auto_hash_equals struct FunctionalVPolicy{
    F, D <: Domain, S <: Specification
} <: PolicySolution
    evaluator::F
    domain::D
    spec::S
end

function FunctionalVPolicy(heuristic::Heuristic,
                           domain::Domain, spec::Specification)
    h_eval(state::State) = -compute(heuristic, domain, state, spec)
    return FunctionalVPolicy(h_eval, domain, spec)
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
