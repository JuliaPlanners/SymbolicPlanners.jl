export StateConstrainedGoal

"Goal specification with state-based constraints."
struct StateConstrainedGoal{G <: Goal} <: Goal
    goal::G
    constraints::Vector{Term}
end

Base.hash(spec::StateConstrainedGoal, h::UInt) =
    hash(spec.constraints, hash(spec.goal, h))
Base.isequal(s1::StateConstrainedGoal, s2::StateConstrainedGoal) =
    Set(s1.constraints) == Set(s2.constraints) && s1.goal == s2.goal

is_goal(spec::StateConstrainedGoal, domain::Domain, state::State) =
    is_goal(spec.goal, domain, state)
is_violated(spec::StateConstrainedGoal, domain::Domain, state::State) =
    !satisfy(spec.constraints, state, domain)[1] ||
    is_violated(spec.goal, domain, state)
get_cost(spec::StateConstrainedGoal, d::Domain, s1::State, a::Term, s2::State) =
    get_cost(spec.goal, d, s1, a, s2)
get_reward(spec::StateConstrainedGoal, d::Domain, s1::State, a::Term, s2::State) =
    get_reward(spec.goal, d, s1, a, s2)
get_goal_terms(spec::StateConstrainedGoal) = get_goal_terms(spec.goal)
