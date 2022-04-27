export MinStepsGoal

"Goal specification where all steps have equal cost."
struct MinStepsGoal <: Goal
    terms::Vector{Term} # Goal terms to be satisfied
end

MinStepsGoal(problem::Problem) =
    MinStepsGoal(PDDL.flatten_conjs(PDDL.get_goal(problem)))
MinStepsGoal(goal::Term) =
    MinStepsGoal(PDDL.flatten_conjs(goal))

Base.hash(spec::MinStepsGoal, h::UInt) =
    hash(Set(spec.terms), h)
Base.:(==)(s1::MinStepsGoal, s2::MinStepsGoal) =
    Set(s1.terms) == Set(s2.terms)

is_goal(spec::MinStepsGoal, domain::Domain, state::State) =
    satisfy(domain, state, spec.terms)
is_violated(spec::MinStepsGoal, domain::Domain, state::State) = false
get_cost(spec::MinStepsGoal, ::Domain, ::State, ::Term, ::State) = 1
get_reward(spec::MinStepsGoal, ::Domain, ::State, ::Term, ::State) = -1
get_goal_terms(spec::MinStepsGoal) = spec.terms
