export MinActionCosts, ExtraActionCosts

"Goal specification with action-specific costs."
struct MinActionCosts{C <: NamedTuple} <: Goal
    terms::Vector{Term} # Goal terms to be satisfied
    costs::C # Named tuple of action costs
end

MinActionCosts(term::Term, costs) =
    MinActionCosts(flatten_conjs(term), costs)

function MinActionCosts(terms, actions, costs)
    costs = NamedTuple{Tuple(actions)}(Tuple(Float64.(costs)))
    return MinActionCosts(flatten_conjs(terms), costs)
end

function MinActionCosts(terms; costs...)
    actions = collect(keys(costs))
    costs = collect(values(costs))
    return MinActionCosts(terms, actions, costs)
end

Base.hash(spec::MinActionCosts, h::UInt) =
    hash(spec.costs, hash(Set(spec.terms), h))
Base.:(==)(s1::MinActionCosts, s2::MinActionCosts) =
    s1.costs == s2.costs && s1.terms == s2.terms

is_goal(spec::MinActionCosts, domain::Domain, state::State) =
    satisfy(domain, state, spec.terms)
is_violated(spec::MinActionCosts, domain::Domain, state::State) = false
get_cost(spec::MinActionCosts, domain::Domain, s1::State, a::Term, s2::State) =
    spec.costs[a.name]
get_reward(spec::MinActionCosts, domain::Domain, s1::State, a::Term, s2::State) =
    -get_cost(spec, domain, s1, a, s2)
get_goal_terms(spec::MinActionCosts) = spec.terms

"Adds action-specific costs to an underlying specification."
struct ExtraActionCosts{S <: Specification, C <: NamedTuple} <: Specification
    spec::S # Underlying specification
    costs::C # Named tuple of action costs
end

function ExtraActionCosts(spec::Specification, actions, costs)
    costs = NamedTuple{Tuple(actions)}(Tuple(Float64.(costs)))
    return ExtraActionCosts(spec, costs)
end

function ExtraActionCosts(spec::Specification; costs...)
    actions = collect(keys(costs))
    costs = collect(values(costs))
    return ExtraActionCosts(spec, actions, costs)
end

Base.hash(spec::ExtraActionCosts, h::UInt) =
    hash(spec.costs, hash(spec.spec, h))
Base.:(==)(s1::ExtraActionCosts, s2::ExtraActionCosts) =
    s1.costs == s2.costs && s1.spec == s2.spec

is_goal(spec::ExtraActionCosts, domain::Domain, state::State) =
    is_goal(spec.spec, domain, state)
is_violated(spec::ExtraActionCosts, domain::Domain, state::State) =
    is_violated(spec.spec, domain, state)
get_cost(spec::ExtraActionCosts, domain::Domain, s1::State, a::Term, s2::State) =
    get_cost(spec.spec, domain, s1, a, s2) + spec.costs[a.name]
get_reward(spec::ExtraActionCosts, domain::Domain, s1::State, a::Term, s2::State) =
    -get_cost(spec, domain, s1, a, s2)
get_goal_terms(spec::ExtraActionCosts) =
    get_goal_terms(spec.spec)
get_discount(spec::ExtraActionCosts) =
    get_discount(spec.spec)
