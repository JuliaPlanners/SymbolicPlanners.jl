export GoalReward, BonusGoalReward, MultiGoalReward

"""
    GoalReward(terms, reward=1.0, discount=0.9)

[`Goal`](@ref) specification which returns a `reward` when all goal `terms`
are achieved, along with a `discount` factor. Each action has zero cost.
"""
@kwdef struct GoalReward <: Goal
    terms::Vector{Term} # Goal terms to be satisfied
    reward::Float64 = 1.0 # Reward gained from reaching goal
    discount::Float64 = 0.9 # Discount factor
end

GoalReward(problem::Problem) = GoalReward(PDDL.get_goal(problem), 1.0, 0.9)
GoalReward(terms, reward) = GoalReward(terms, reward, 0.9)
GoalReward(terms) = GoalReward(terms, 1.0, 0.9)
GoalReward(term::Term, reward, discount) =
    GoalReward(PDDL.flatten_conjs(term), reward, discount)

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

set_goal_terms(spec::GoalReward, terms) =
    GoalReward(terms, spec.reward, spec.discount)

discounted(spec::GoalReward, discount::Float64) =
    GoalReward(spec.terms, spec.reward, discount * spec.discount)


"""
    BonusGoalReward(goal::Goal, reward=1.0, discount=0.9)

Wrapper around an existing [`Goal`](@ref) specification, which delivers
additional `reward` upon reaching a goal.
"""
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
    (is_goal(spec, domain, s2) ? spec.reward : 0.0)
get_discount(spec::BonusGoalReward) = spec.discount * get_discount(spec.goal)
get_goal_terms(spec::BonusGoalReward) = get_goal_terms(spec.goal)

set_goal_terms(spec::BonusGoalReward, terms) =
    BonusGoalReward(set_goal_terms(spec.goal, terms), spec.reward, spec.discount)

discounted(spec::BonusGoalReward, discount::Float64) =
    BonusGoalReward(spec.goal, spec.reward, discount * spec.discount)

"""
    MultiGoalReward(goals::Vector{Term}, rewards::Vector{Float64}, discount=1.0)

[`Goal`](@ref) specification where multiple `goals` have associated `rewards`.
Achieving a goal delivers the associated reward. Each action has zero cost.
"""
@kwdef struct MultiGoalReward <: Goal
    goals::Vector{Term} # List of possible goals
    rewards::Vector{Float64} # Rewards gained from reaching each goal
    discount::Float64 = 1.0 # Discount factor
end

MultiGoalReward(goals, rewards) = MultiGoalReward(goals, rewards, 1.0)

Base.hash(spec::MultiGoalReward, h::UInt) =
    hash(spec.rewards, hash(spec.discount, hash(spec.goals, h)))
Base.:(==)(s1::MultiGoalReward, s2::MultiGoalReward) =
    s1.discount == s2.discount && s1.rewards == s2.rewards &&
    s1.goals == s2.goals

is_goal(spec::MultiGoalReward, domain::Domain, state::State) =
    any(satisfy(domain, state, g) for g in spec.goals)
is_violated(spec::MultiGoalReward, domain::Domain, state::State) = false
get_cost(spec::MultiGoalReward, domain::Domain, s1::State, a::Term, s2::State) =
    -get_reward(spec, domain, s1, a, s2)
get_discount(spec::MultiGoalReward) = spec.discount

function get_reward(spec::MultiGoalReward, domain::Domain,
                    s1::State, a::Term, s2::State)
    reward = 0.0
    for (goal, r) in zip(spec.goals, spec.rewards)
        if satisfy(domain, s2, goal) reward += r end
    end
    return reward
end

get_goal_terms(spec::MultiGoalReward) =
    Term[Compound(:or, spec.goals)]

function set_goal_terms(spec::MultiGoalReward, terms)
    goals = PDDL.flatten_disjs(terms)
    if length(goals) != length(spec.rewards)
        error("Number of goals must match number of rewards.")
    end
    return MultiGoalReward(goals, spec.rewards, spec.discount)
end

discounted(spec::MultiGoalReward, discount::Float64) =
    MultiGoalReward(spec.goals, spec.rewards, discount * spec.discount)
