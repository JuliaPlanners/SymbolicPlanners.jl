export DiscountedReward, discounted

"Discounts reward of underlying specification."
struct DiscountedReward{S <: Specification} <: Specification
    spec::S # Underlying specification
    discount::Float64 # Discount factor
end

DiscountedReward(spec) = DiscountedReward(spec, 1.0, 1.0)
DiscountedReward(spec::DiscountedReward, discount) =
    DiscountedReward(spec.spec, discount * spec.discount)

Base.hash(spec::DiscountedReward, h::UInt) =
    hash(spec.discount, hash(spec.spec, h))
Base.:(==)(s1::DiscountedReward, s2::DiscountedReward) =
    s1.discount == s2.discount && s1.spec == s2.spec

is_goal(spec::DiscountedReward, domain::Domain, state::State) =
    is_goal(spec.spec, domain, state)
is_violated(spec::DiscountedReward, domain::Domain, state::State) =
    is_violated(spec.spec, domain, state)
get_cost(spec::DiscountedReward, domain::Domain, s1::State, a::Term, s2::State) =
    get_cost(spec.spec, domain, s1, a, s2)
get_reward(spec::DiscountedReward, domain::Domain, s1::State, a::Term, s2::State) =
    get_reward(spec.spec, domain, s1, a, s2)
get_goal_terms(spec::DiscountedReward) =
    get_goal_terms(spec.spec)

"Discount the rewards or costs associated with `spec`."
discounted(spec::Specification, discount::Float64) =
    DiscountedReward(spec, discount)
