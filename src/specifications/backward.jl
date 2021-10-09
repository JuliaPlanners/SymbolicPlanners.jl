"Goal specification for backward search."
struct BackwardSearchGoal{G <: Goal,C} <: Goal
    goal::G # Original goal specification
    start::State # Start state to be reached via backward search
    constraint_diff::C # State constraints as a diff, if any
end

BackwardSearchGoal(goal::Goal, start::State) =
    BackwardSearchGoal(goal, start, nothing)
BackwardSearchGoal(goal::StateConstrainedGoal, start::State) =
    BackwardSearchGoal(goal, start, precond_diff(constraints))

Base.hash(spec::BackwardSearchGoal, h::UInt) =
    hash(spec.start, hash(spec.goal, h))
Base.:(==)(s1::BackwardSearchGoal, s2::BackwardSearchGoal) =
    s1.state == s2.state && s1.goal == s2.goal

is_goal(spec::BackwardSearchGoal, domain::Domain, state::State) =
    issubset(state, spec.start)
is_violated(spec::BackwardSearchGoal, domain::Domain, state::State) =
    is_violated(spec.goal, domain, state)
get_cost(spec::BackwardSearchGoal, domain::Domain, s1::State, a::Term, s2::State) =
    get_cost(spec.goal, domain, s2, a, s1)
get_reward(spec::BackwardSearchGoal, domain::Domain, s1::State, a::Term, s2::State) =
    get_reward(spec.goal, domain, s2, a, s1)
get_goal_terms(spec::BackwardSearchGoal) =
    get_goal_terms(spec.goal)

add_constraints!(spec::BackwardSearchGoal, state::State) =
    nothing
add_constraints!(spec::BackwardSearchGoal{G,PDDL.Diff}, state::State) where {G} =
    update!(state, spec.constraint_diff)
