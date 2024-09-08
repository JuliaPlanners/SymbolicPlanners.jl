export MinStepsGoal

"""
    MinStepsGoal(terms)
    MinStepsGoal(goal::Term)
    MinStepsGoal(problem::Problem)

[`Goal`](@ref) specification where each step (i.e. action) has unit cost, 
and the goal formula is a conjunction of `terms`. Planners called with this
specification will try to minimize the number of steps to the goal in the
returned [`Solution`](@ref).
"""
struct MinStepsGoal <: Goal
    terms::Vector{Term} # Goal terms to be satisfied
end

MinStepsGoal(problem::Problem) =
    MinStepsGoal(PDDL.flatten_conjs(PDDL.get_goal(problem)))
MinStepsGoal(goal::Term) =
    MinStepsGoal(PDDL.flatten_conjs(goal))

function Base.show(io::IO, ::MIME"text/plain", spec::MinStepsGoal)
    indent = get(io, :indent, "")
    show_struct(io, spec; indent = indent, show_pddl_list=(:terms,))
end
    
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

set_goal_terms(spec::MinStepsGoal, terms) =
    MinStepsGoal(terms)