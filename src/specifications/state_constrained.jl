export StateConstrainedGoal

"""
    StateConstrainedGoal(goal::Specification, constraints::Vector{Term})

[`Goal`](@ref) specification with a list of `constraints` that must hold
for every state. Planners that receive this specification are required to return
plans or policies that ensure every visited state satisfies the constraints.
"""
struct StateConstrainedGoal{S <: Specification} <: Goal
    goal::S
    constraints::Vector{Term}
end

function StateConstrainedGoal(problem::Problem)
    # Convert problem to underlying goal
    goal = Specification(problem)
    # Get constraints
    constraints = PDDL.get_constraints(problem)::Term
    return StateConstrainedGoal(goal, constraints)
end
StateConstrainedGoal(goal::Specification, constraints::Term) =
    StateConstrainedGoal(goal, PDDL.flatten_conjs(constraints))

function Base.show(io::IO, ::MIME"text/plain", spec::StateConstrainedGoal)
    indent = get(io, :indent, "")
    show_struct(io, spec; indent = indent, show_fields=(:goal,),
                show_pddl_list=(:constraints,))
end

Base.hash(spec::StateConstrainedGoal, h::UInt) =
    hash(Set(spec.constraints), hash(spec.goal, h))
Base.:(==)(s1::StateConstrainedGoal, s2::StateConstrainedGoal) =
    Set(s1.constraints) == Set(s2.constraints) && s1.goal == s2.goal

is_violated(spec::StateConstrainedGoal, domain::Domain, state::State) =
    !satisfy(domain, state, spec.constraints) ||
    is_violated(spec.goal, domain, state)
get_cost(spec::StateConstrainedGoal, d::Domain, s1::State, a::Term, s2::State) =
    is_violated(spec.goal, d, s2) ? Inf : get_cost(spec.goal, d, s1, a, s2)
get_reward(spec::StateConstrainedGoal, d::Domain, s1::State, a::Term, s2::State) =
    is_violated(spec.goal, d, s2) ? -Inf : get_reward(spec.goal, d, s1, a, s2)

@set_subspec(StateConstrainedGoal, goal)
