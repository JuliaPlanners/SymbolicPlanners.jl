export DiscountedReward, discounted

"""
    DiscountedReward(spec::Specification, discount::Float64)

Discounts rewards of the underlying `spec` by a `discount` factor.
"""
struct DiscountedReward{S <: Specification} <: Specification
    spec::S # Underlying specification
    discount::Float64 # Discount factor
end

DiscountedReward(spec) = DiscountedReward(spec, 1.0)
DiscountedReward(spec::DiscountedReward, discount::Real) =
    DiscountedReward(spec.spec, discount * spec.discount)
DiscountedReward(spec::DiscountedReward, discount::Float64) =
    DiscountedReward(spec.spec, discount * spec.discount)

function Base.show(io::IO, ::MIME"text/plain", spec::DiscountedReward)
    indent = get(io, :indent, "")
    show_struct(io, spec; indent = indent, show_fields=(:spec,))
end

Base.hash(spec::DiscountedReward, h::UInt) =
    hash(spec.discount, hash(spec.spec, h))
Base.:(==)(s1::DiscountedReward, s2::DiscountedReward) =
    s1.discount == s2.discount && s1.spec == s2.spec

is_goal(spec::DiscountedReward, domain::Domain, state::State) =
    is_goal(spec.spec, domain, state)
is_goal(spec::DiscountedReward, domain::Domain, state::State, action::Term) =
    is_goal(spec.spec, domain, state, action)
is_violated(spec::DiscountedReward, domain::Domain, state::State) =
    is_violated(spec.spec, domain, state)
get_cost(spec::DiscountedReward, domain::Domain, s1::State, a::Term, s2::State) =
    get_cost(spec.spec, domain, s1, a, s2)
get_reward(spec::DiscountedReward, domain::Domain, s1::State, a::Term, s2::State) =
    get_reward(spec.spec, domain, s1, a, s2)
get_discount(spec::DiscountedReward) =
    spec.discount * get_discount(spec.spec)
get_goal_terms(spec::DiscountedReward) =
    get_goal_terms(spec.spec)

set_goal_terms(spec::DiscountedReward, terms) =
    DiscountedReward(set_goal_terms(spec.spec, terms), spec.discount)

has_action_goal(spec::DiscountedReward) = has_action_goal(spec.spec)
has_action_cost(spec::DiscountedReward) = has_action_cost(spec.spec)
get_action_cost(spec::DiscountedReward, action::Term) =
    get_action_cost(spec.spec, action)

"""
$(SIGNATURES)

Discount the rewards or costs associated with `spec` by a `discount` factor.
"""
discounted(spec::Specification, discount::Float64) =
    DiscountedReward(spec, discount)
