export MinActionCosts, ExtraActionCosts
export infer_action_costs

"""
$(SIGNATURES)

Returns whether a specification has action-specific costs.
"""
has_action_cost(spec::Specification) = false

"""
$(SIGNATURES)

Returns the cost for `act` for specifications with fixed action costs.
"""
get_action_cost(spec::Specification, act::Term) =
    error("Not implemented.")

"""
    MinActionCosts(terms, costs)
    MinActionCosts(terms, actions, costs)

[`Goal`](@ref) specification where each action has a specific cost, and the goal
formula is a conjunction of `terms`. Planners called with this specification
will try to minimize the total action cost in the returned [`Solution`](@ref).

Costs can be provided as mapping from action names (specified as `Symbol`s)
to `Real`s, such that each lifted action has an associated cost. Alternatively,
costs can be provided as a mapping from ground action `Term`s to `Real`s.
A mapping can be provided directly as a `NamedTuple` or `Dictionary`, or as
a list of `actions` and corresponding `costs`.
"""
struct MinActionCosts{C} <: Goal
    terms::Vector{Term} # Goal terms to be satisfied
    costs::C # Named tuple or dictionary of action costs
end

MinActionCosts(term::Term, costs) =
    MinActionCosts(PDDL.flatten_conjs(term), costs)

MinActionCosts(terms::AbstractVector{<:Term}, costs) =
    MinActionCosts(collect(Term, terms), costs)

function MinActionCosts(terms::AbstractVector{<:Term},
                        actions::AbstractVector{<:Symbol},
                        costs::AbstractVector{<:Real})
    costs = NamedTuple{Tuple(actions)}(Tuple(float.(costs)))
    return MinActionCosts(PDDL.flatten_conjs(terms), costs)
end

function MinActionCosts(terms::AbstractVector{<:Term},
                        actions::AbstractVector{<:Term},
                        costs::AbstractVector{<:Real})
    costs = Dict(zip(actions, costs)...)
    return MinActionCosts(PDDL.flatten_conjs(terms), costs)
end

function MinActionCosts(terms; costs...)
    actions = collect(keys(costs))
    costs = collect(values(costs))
    return MinActionCosts(terms, actions, costs)
end

"""
    MinActionCosts(domain::Domain, problem::Problem)

Attempts to infer action costs from a `domain` and `problem`, and constructs
a corresponding specification if successful.
"""
function MinActionCosts(domain::Domain, problem::Problem)
    costs = infer_action_costs(domain, problem)
    if costs === nothing
        error("Action costs could not be inferred from domain and problem.")
    end
    return MinActionCosts(PDDL.get_goal(problem), costs)
end

Base.hash(spec::MinActionCosts, h::UInt) =
    hash(spec.costs, hash(Set(spec.terms), h))
Base.:(==)(s1::MinActionCosts, s2::MinActionCosts) =
    s1.costs == s2.costs && s1.terms == s2.terms

has_action_cost(spec::MinActionCosts) = true

get_action_cost(spec::MinActionCosts{C}, act::Term) where {C <: NamedTuple} =
    spec.costs[act.name]
get_action_cost(spec::MinActionCosts{C}, act::Term) where {C <: Dict{Symbol}} =
    spec.costs[act.name]
get_action_cost(spec::MinActionCosts{C}, act::Term) where {C <: Dict{<:Term}} =
    spec.costs[act]

is_goal(spec::MinActionCosts, domain::Domain, state::State) =
    satisfy(domain, state, spec.terms)
is_violated(spec::MinActionCosts, domain::Domain, state::State) = false
get_cost(spec::MinActionCosts, domain::Domain, s1::State, a::Term, s2::State) =
    get_action_cost(spec, a)
get_reward(spec::MinActionCosts, domain::Domain, s1::State, a::Term, s2::State) =
    -get_action_cost(spec, a)
get_goal_terms(spec::MinActionCosts) = spec.terms

set_goal_terms(spec::MinActionCosts, terms) =
    MinActionCosts(terms, spec.costs)

"""
    ExtraActionCosts(spec::Specification, costs)
    ExtraActionCosts(spec::Specification, actions, costs)

Wrapper that adds action-specific costs to an underlying `spec`.

Costs can be provided as mapping from action names (specified as `Symbol`s)
to `Real`s, such that each lifted action has an associated cost. Alternatively,
costs can be provided as a mapping from ground action `Term`s to `Real`s.
A mapping can be provided directly as a `NamedTuple` or `Dictionary`, or as
a list of `actions` and corresponding `costs`.
"""
struct ExtraActionCosts{S <: Specification, C} <: Specification
    spec::S # Underlying specification
    costs::C # Named tuple or dictionary of action costs
end

function ExtraActionCosts(spec::Specification,
                          actions::AbstractVector{<:Symbol},
                          costs::AbstractVector{<:Real})
    costs = NamedTuple{Tuple(actions)}(Tuple(float.(costs)))
    return ExtraActionCosts(spec, costs)
end

function ExtraActionCosts(spec::Specification,
                          actions::AbstractVector{<:Term},
                          costs::AbstractVector{<:Real})
    costs = Dict(zip(actions, costs)...)
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

has_action_cost(spec::ExtraActionCosts) = true

get_action_cost(spec::ExtraActionCosts{C}, act::Term) where {C <: NamedTuple} =
    spec.costs[act.name]
get_action_cost(spec::ExtraActionCosts{C}, act::Term) where {C <: Dict{Symbol}} =
    spec.costs[act.name]
get_action_cost(spec::ExtraActionCosts{C}, act::Term) where {C <: Dict{<:Term}} =
    spec.costs[act]

is_goal(spec::ExtraActionCosts, domain::Domain, state::State) =
    is_goal(spec.spec, domain, state)
is_violated(spec::ExtraActionCosts, domain::Domain, state::State) =
    is_violated(spec.spec, domain, state)
get_cost(spec::ExtraActionCosts, domain::Domain, s1::State, a::Term, s2::State) =
    get_cost(spec.spec, domain, s1, a, s2) + get_action_cost(a)
get_reward(spec::ExtraActionCosts, domain::Domain, s1::State, a::Term, s2::State) =
    -get_cost(spec, domain, s1, a, s2)
get_goal_terms(spec::ExtraActionCosts) =
    get_goal_terms(spec.spec)
get_discount(spec::ExtraActionCosts) =
    get_discount(spec.spec)

set_goal_terms(spec::ExtraActionCosts, terms) =
    ExtraActionCosts(set_goal_terms(spec.spec, terms), spec.costs)

"""
$(SIGNATURES)

Infer fixed action costs for a `domain` and `problem`, returning `nothing`
if unsuccessful.
"""
function infer_action_costs(domain::Domain, problem::Problem)
    # Extract fluents in metric expression
    metric = PDDL.get_metric(problem)
    if metric === nothing || metric.name != :minimize
        return nothing
    end
    metric = metric.args[1]
    state = initstate(domain, problem)
    # Infer action costs given state and metric
    return infer_action_costs(domain, state, metric)
end

"""
    infer_action_costs(
        domain::Domain, state::State, metric::Term,
        cost_fluents=PDDL.constituents(metric, domain),
        static_fluents=infer_static_fluents(domain)
    )

Infer fixed action costs for a `domain` and initial `state`, and `metric`
formula, returning `nothing` if unsuccessful.
"""
function infer_action_costs(
    domain::Domain, state::State, metric::Term,
    cost_fluents=PDDL.constituents(metric, domain),
    static_fluents=infer_static_fluents(domain)
)
    # Determine costs for each action schema, if possible
    costs = infer_lifted_action_costs(domain, state, metric,
                                      cost_fluents, static_fluents)
    if (costs !== nothing) return costs end
    # Determine action costs for each ground action
    costs = infer_ground_action_costs(domain, state, metric,
                                      cost_fluents, static_fluents)
    return costs
end

"""
    infer_lifted_action_costs(
        domain::Domain, state::State, metric::Term,
        cost_fluents=PDDL.constituents(metric, domain),
        static_fluents=infer_static_fluents(domain)
    )

Infer costs for each lifted action, returning `nothing` if unsuccessful.
"""
function infer_lifted_action_costs(
    domain::Domain, state::State, metric::Term,
    cost_fluents=PDDL.constituents(metric, domain),
    static_fluents=infer_static_fluents(domain)
)
    actions = Symbol[]
    costs = Float64[]
    for (name, act) in PDDL.get_actions(domain)
        diff = PDDL.effect_diff(domain, state, PDDL.get_effect(act))
        act_cost = Float64(0)
        for f in cost_fluents
            op_expr = get(diff.ops, f, nothing)
            op_expr !== nothing || continue
            val_expr = _get_increase_expr(op_expr) # Get value of increment
            if (!is_ground(op_expr) || val_expr === nothing ||
                !PDDL.is_static(val_expr, domain, static_fluents))
                return nothing
            end
            val = evaluate(domain, state, val_expr)
            act_cost += val
        end
        push!(actions, name)
        push!(costs, act_cost)
    end
    costs = NamedTuple{Tuple(actions)}(Tuple(costs))
    return costs
end

"""
    infer_ground_action_costs(
        domain::Domain, state::State, metric::Term,
        cost_fluents=PDDL.constituents(metric, domain),
        static_fluents=infer_static_fluents(domain)
    )

Infer costs for each ground action, returning `nothing` if unsuccessful.
"""
function infer_ground_action_costs(
    domain::Domain, state::State, metric::Term,
    cost_fluents=PDDL.constituents(metric, domain),
    static_fluents=infer_static_fluents(domain)
)
    costs = Dict{Term,Float64}()
    for act in groundactions(domain, state)
        diff = PDDL.effect_diff(domain, state, PDDL.get_effect(act))
        act_cost = Float64(0)
        for f in cost_fluents
            op_expr = get(diff.ops, f, nothing)
            op_expr !== nothing || continue
            val_expr = _get_increase_expr(op_expr) # Get value of increment
            if (val_expr === nothing ||
                !PDDL.is_static(val_expr, domain, static_fluents))
                return nothing
            end
            val = evaluate(domain, state, val_expr)
            act_cost += val
        end
        costs[act.term] = act_cost
    end
    return costs
end

function _get_increase_expr(term::Term)
    if term.name == :+
        return term.args[2]
    elseif term.name == :-
        return Compound(:-, [term.args[2]])
    else
        return nothing
    end
end
