export FunctionalVPolicy, FunctionalQPolicy

"Policy solution where (state) values are evaluated by a function."
struct FunctionalVPolicy{F, D <: Domain, S <: Specification} <: PolicySolution
    evaluator::F
    domain::D
    spec::S
end

function FunctionalVPolicy(heuristic::Heuristic,
                           domain::Domain, spec::Specification)
    h_eval(state::State) = -compute(heuristic, domain, state, spec)
    return FunctionalVPolicy(h_eval, domain, spec)
end

get_action(sol::FunctionalVPolicy, state::State) =
    best_action(sol, state)
rand_action(sol::FunctionalVPolicy, state::State) =
    best_action(sol, state)
best_action(sol::FunctionalVPolicy, state::State) =
    first(reduce((a, b) -> last(a) > last(b) ? a : b,
                  get_action_values(sol, state)))
get_value(sol::FunctionalVPolicy, state::State) =
    sol.evaluator(state)
get_action_values(sol::FunctionalVPolicy, state::State) =
    (act => get_value(sol, state, act) for act in available(sol.domain, state))

function get_value(sol::FunctionalVPolicy, state::State, action::Term)
    next_state = transition(sol.domain, state, action)
    next_v = sol.evaluator(next_state)
    r = get_reward(sol.spec, sol.domain, state, action, next_state)
    return get_discount(sol.spec) * next_v + r
end
