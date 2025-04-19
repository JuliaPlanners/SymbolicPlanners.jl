export BackwardSearchGoal

"""
    BackwardSearchGoal(goal::Goal, start::State)

A goal specification used for backward search (e.g. [`BackwardPlanner`](@ref)),
constructed from the original `goal` specification, and the `start` state to
be reached via backward search from the goal.
"""
struct BackwardSearchGoal{G <: Goal, C} <: Goal
    goal::G # Original goal specification
    start::State # Start state to be reached via backward search
    constraint_diff::C # State constraints as a diff, if any
end

BackwardSearchGoal(goal::Goal, start::State) =
    BackwardSearchGoal(goal, start, nothing)
BackwardSearchGoal(goal::Goal, domain::Domain, start::State) =
    BackwardSearchGoal(goal, start, nothing)

function BackwardSearchGoal(goal::StateConstrainedGoal,
                            domain::Domain, start::State)
    constraint = Compound(:and, goal.constraints)
    constraint_diff = PDDL.precond_diff(domain, start, constraint)
    BackwardSearchGoal(goal, start, constraint_diff)
end

function Base.show(io::IO, ::MIME"text/plain", spec::BackwardSearchGoal)
    indent = get(io, :indent, "")
    show_struct(io, spec; indent = indent, show_fields=(:goal,))
end

Base.hash(spec::BackwardSearchGoal, h::UInt) =
    hash(spec.start, hash(spec.goal, h))
Base.:(==)(s1::BackwardSearchGoal, s2::BackwardSearchGoal) =
    s1.start == s2.start && s1.goal == s2.goal

is_goal(spec::BackwardSearchGoal, domain::Domain, state::State) =
    issubset(state, spec.start)

function add_constraints!(
    spec::BackwardSearchGoal{G,C}, domain::Domain, state::State
) where {G,C <: PDDL.Diff}
    update!(state, domain, spec.constraint_diff)
end
add_constraints!(spec::BackwardSearchGoal, domain::Domain, state::State) =
    nothing

@set_subspec(BackwardSearchGoal, goal)
