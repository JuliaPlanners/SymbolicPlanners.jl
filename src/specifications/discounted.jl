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

get_discount(spec::DiscountedReward) =
    spec.discount * get_discount(spec.spec)

@set_subspec(DiscountedReward, spec)

"""
$(SIGNATURES)

Discount the rewards or costs associated with `spec` by a `discount` factor.
"""
discounted(spec::Specification, discount::Float64) =
    DiscountedReward(spec, discount)
