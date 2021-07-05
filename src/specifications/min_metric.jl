export MinMetricGoal

"Goal specification where costs are differences in a state-based metric."
struct MinMetricGoal <: Goal
    terms::Vector{Term} # Goal terms to be satisfied
    metric::Term # Cost metric to be minimized
end

function MinMetricGoal(problem::Problem)
    goals = flatten_conjs(problem.goal)
    sign, metric = problem.metric
    if sign > 0 metric = Compound(:-, [metric]) end
    return MinMetricGoal(goals, metric)
end
MinMetricGoal(goal::Term, metric::Term) =
    MinMetricGoal(flatten_conjs(goal), metric)

Base.hash(spec::MinMetricGoal, h::UInt) =
    hash(spec.metric, hash(Set(spec.terms), h))
Base.isequal(s1::MinMetricGoal, s2::MinMetricGoal) =
    s1.metric == s2.metric && Set(s1.terms) == Set(s2.terms)

is_goal(spec::MinMetricGoal, domain::Domain, state::State) =
    satisfy(spec.terms, state, domain)[1]
is_violated(spec::MinMetricGoal, domain::Domain, state::State) = false
get_cost(spec::MinMetricGoal, domain::Domain, s1::State, ::Term, s2::State) =
    s2[domain, spec.metric] - s1[domain, spec.metric]
get_reward(spec::MinMetricGoal, domain::Domain, s1::State, ::Term, s2::State) =
    s1[domain, spec.metric] - s2[domain, spec.metric]
get_goal_terms(spec::MinMetricGoal) = spec.terms
