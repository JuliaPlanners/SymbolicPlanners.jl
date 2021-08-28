export MaxMetricGoal

"Goal specification where costs are differences in a state-based metric."
struct MaxMetricGoal <: Goal
    terms::Vector{Term} # Goal terms to be satisfied
    metric::Term # Reward metric to be maximized
end

function MaxMetricGoal(problem::Problem)
    goals = flatten_conjs(problem.goal)
    sign, metric = problem.metric
    if sign > 0 metric = Compound(:-, [metric]) end
    return MaxMetricGoal(goals, metric)
end
MaxMetricGoal(goal::Term, metric::Term) =
    MaxMetricGoal(flatten_conjs(goal), metric)

Base.hash(spec::MaxMetricGoal, h::UInt) =
    hash(spec.metric, hash(Set(spec.terms), h))
Base.isequal(s1::MaxMetricGoal, s2::MaxMetricGoal) =
    s1.metric == s2.metric && Set(s1.terms) == Set(s2.terms)

is_goal(spec::MaxMetricGoal, domain::Domain, state::State) =
    satisfy(domain, state, spec.terms)
is_violated(spec::MaxMetricGoal, domain::Domain, state::State) = false
get_cost(spec::MaxMetricGoal, domain::Domain, s1::State, ::Term, s2::State) =
    domain[s1 => spec.metric] - domain[s2 => spec.metric]
get_reward(spec::MaxMetricGoal, domain::Domain, s1::State, ::Term, s2::State) =
    domain[s2 => spec.metric] - domain[s1 => spec.metric]
get_goal_terms(spec::MaxMetricGoal) = spec.terms
