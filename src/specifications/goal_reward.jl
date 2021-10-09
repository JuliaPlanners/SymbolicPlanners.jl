export GoalReward, BonusGoalReward

"Specification where reaching the goal delivers reward."
@kwdef struct GoalReward <: Goal
    terms::Vector{Term} # Goal terms to be satisfied
    reward::Float64 = 1.0 # Reward gained from reaching goal
    discount::Float64 = 0.9 # Discount factor
end

GoalReward(problem::Problem) = GoalReward(flatten_conjs(problem.goal), 1.0, 0.9)
GoalReward(terms, reward) = GoalReward(terms, reward, 0.9)
GoalReward(terms) = GoalReward(terms, 1.0, 0.9)
GoalReward(term::Term, reward, discount) =
    GoalReward(flatten_conjs(term), reward, discount)

Base.hash(spec::GoalReward, h::UInt) =
    hash(spec.reward, hash(spec.discount, hash(Set(spec.terms), h)))
Base.:(==)(s1::GoalReward, s2::GoalReward) =
    s1.reward == s2.reward && s1.discount == s2.discount &&
    Set(s1.terms) == Set(s2.terms)

is_goal(spec::GoalReward, domain::Domain, state::State) =
    satisfy(domain, state, spec.terms)
is_violated(spec::GoalReward, domain::Domain, state::State) = false
get_cost(spec::GoalReward, domain::Domain, s1::State, a::Term, s2::State) =
    -get_reward(spec, domain, s1, a, s2)
get_reward(spec::GoalReward, domain::Domain, s1::State, a::Term, s2::State) =
    is_goal(spec, domain, s2) ? spec.reward : 0.0
get_discount(spec::GoalReward) = spec.discount
get_goal_terms(spec::GoalReward) = spec.terms

discounted(spec::GoalReward, discount::Float64) =
    GoalReward(spec.terms, spec.reward, discount * spec.discount)

"Wrapper specification that delivers additional reward upon reaching goal."
@kwdef struct BonusGoalReward{G <: Goal} <: Goal
    goal::G # Goal specification to be satisfied
    reward::Float64 = 1.0 # Additional reward gained from reaching goal
    discount::Float64 = 1.0 # Discount factor
end

BonusGoalReward(spec::Goal) = BonusGoalReward(spec, 1.0, 1.0)
BonusGoalReward(spec::Goal, reward) = BonusGoalReward(spec, reward, 1.0)
BonusGoalReward(spec::BonusGoalReward, reward, discount) =
    BonusGoalReward(spec.goal, reward + spec.reward, discount * spec.discount)

is_goal(spec::BonusGoalReward, domain::Domain, state::State) =
    is_goal(spec.goal, domain, state)
is_violated(spec::BonusGoalReward, domain::Domain, state::State) =
    is_violated(spec.goal, domain, state)
get_cost(spec::BonusGoalReward, domain::Domain, s1::State, a::Term, s2::State) =
    -get_reward(spec, domain, s1, a, s2)
get_reward(spec::BonusGoalReward, domain::Domain, s1::State, a::Term, s2::State) =
    get_reward(spec.goal, domain, s1, a, s2) +
    is_goal(spec, domain, s2) ? spec.reward : 0.0
get_discount(spec::BonusGoalReward) = spec.discount
get_goal_terms(spec::BonusGoalReward) = get_goal_terms(spec.goal)

discounted(spec::BonusGoalReward, discount::Float64) =
    BonusGoalReward(spec.goal, spec.reward, discount * spec.discount)
