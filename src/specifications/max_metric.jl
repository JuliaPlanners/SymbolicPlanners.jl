export MaxMetricGoal

"""
    MaxMetricGoal(goals, metric::Term)
    MaxMetricGoal(goal::Term, metric::Term)
    MaxMetricGoal(problem::Problem)

[`Goal`](@ref) specification where each step has a reward specified by the 
difference in values of a `metric` formula between the next state and the
current state, and the goal formula is a conjuction of `terms`.Planners called
with this specification will try to maximize the `metric` formula when solving
for the goal.
"""
struct MaxMetricGoal <: Goal
    terms::Vector{Term} # Goal terms to be satisfied
    metric::Term # Reward metric to be maximized
end

function MaxMetricGoal(problem::Problem)
    goals = PDDL.flatten_conjs(PDDL.get_goal(problem))
    metric = PDDL.get_metric(problem)
    metric = metric.name == :maximize ?
        metric.args[1] : Compound(:-, metric.args)
    return MaxMetricGoal(goals, metric)
end
MaxMetricGoal(goal::Term, metric::Term) =
    MaxMetricGoal(PDDL.flatten_conjs(goal), metric)

function Base.show(io::IO, ::MIME"text/plain", spec::MaxMetricGoal)
    indent = get(io, :indent, "")
    show_struct(io, spec; indent = indent,
                show_pddl_list=(:terms,), show_pddl=(:metric,))
end

Base.hash(spec::MaxMetricGoal, h::UInt) =
    hash(spec.metric, hash(Set(spec.terms), h))
Base.:(==)(s1::MaxMetricGoal, s2::MaxMetricGoal) =
    s1.metric == s2.metric && Set(s1.terms) == Set(s2.terms)

is_goal(spec::MaxMetricGoal, domain::Domain, state::State) =
    satisfy(domain, state, spec.terms)
is_violated(spec::MaxMetricGoal, domain::Domain, state::State) = false
get_cost(spec::MaxMetricGoal, domain::Domain, s1::State, ::Term, s2::State) =
    domain[s1 => spec.metric] - domain[s2 => spec.metric]
get_reward(spec::MaxMetricGoal, domain::Domain, s1::State, ::Term, s2::State) =
    domain[s2 => spec.metric] - domain[s1 => spec.metric]
get_goal_terms(spec::MaxMetricGoal) = spec.terms

set_goal_terms(spec::MaxMetricGoal, terms) =
    MaxMetricGoal(terms, spec.metric)