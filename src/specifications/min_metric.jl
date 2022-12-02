export MinMetricGoal

"Goal specification where costs are differences in a state-based metric."
struct MinMetricGoal <: Goal
    terms::Vector{Term} # Goal terms to be satisfied
    metric::Term # Cost metric to be minimized
end

function MinMetricGoal(problem::Problem)
    goals = PDDL.flatten_conjs(PDDL.get_goal(problem))
    metric = PDDL.get_metric(problem)
    metric = metric.name == :minimize ?
        metric.args[1] : Compound(:-, metric.args)
    return MinMetricGoal(goals, metric)
end
MinMetricGoal(goal::Term, metric::Term) =
    MinMetricGoal(PDDL.flatten_conjs(goal), metric)

Base.hash(spec::MinMetricGoal, h::UInt) =
    hash(spec.metric, hash(Set(spec.terms), h))
Base.:(==)(s1::MinMetricGoal, s2::MinMetricGoal) =
    s1.metric == s2.metric && Set(s1.terms) == Set(s2.terms)

is_goal(spec::MinMetricGoal, domain::Domain, state::State) =
    satisfy(domain, state, spec.terms)
is_violated(spec::MinMetricGoal, domain::Domain, state::State) = false
get_cost(spec::MinMetricGoal, domain::Domain, s1::State, ::Term, s2::State) =
    domain[s2 => spec.metric] - domain[s1 => spec.metric]
get_reward(spec::MinMetricGoal, domain::Domain, s1::State, ::Term, s2::State) =
    domain[s1 => spec.metric] - domain[s2 => spec.metric]
get_goal_terms(spec::MinMetricGoal) = spec.terms

set_goal_terms(spec::MinMetricGoal, terms) =
    MinMetricGoal(terms, spec.metric)