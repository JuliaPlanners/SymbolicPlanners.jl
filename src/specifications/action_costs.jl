export MinActionCosts, ExtraActionCosts

"Returns whether a specification has action-specific costs."
has_action_cost(spec::Specification) = false

"Returns the cost for `act` for specifications with fixed action costs."
get_action_cost(spec::Specification, act::Term) =
    error("Not implemented.")

"Goal specification with action-specific costs."
struct MinActionCosts{C} <: Goal
    terms::Vector{Term} # Goal terms to be satisfied
    costs::C # Named tuple or dictionary of action costs
end

MinActionCosts(term::Term, costs) =
    MinActionCosts(flatten_conjs(term), costs)

function MinActionCosts(terms::AbstractVector{<:Term},
                        actions::AbstractVector{<:Symbol},
                        costs::AbstractVector{<:Real})
    costs = NamedTuple{Tuple(actions)}(Tuple(float.(costs)))
    return MinActionCosts(flatten_conjs(terms), costs)
end

function MinActionCosts(terms::AbstractVector{<:Term},
                        actions::AbstractVector{<:Term},
                        costs::AbstractVector{<:Real})
    costs = Dict(zip(actions, costs)...)
    return MinActionCosts(flatten_conjs(terms), costs)
end

function MinActionCosts(terms; costs...)
    actions = collect(keys(costs))
    costs = collect(values(costs))
    return MinActionCosts(terms, actions, costs)
end

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

"Adds action-specific costs to an underlying specification."
struct ExtraActionCosts{S <: Specification, C} <: Specification
    spec::S # Underlying specification
    costs::C # Named tuple of action costs
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

"Infer fixed action costs, returning `nothing` if unsuccessful."
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

"Infer costs for each lifted action, returning `nothing` if unsuccessful."
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

"Infer costs for each ground action, returning `nothing` if unsuccessful."
function infer_ground_action_costs(
    domain::Domain, state::State, metric::Term,
    cost_fluents=PDDL.constituents(metric, domain),
    static_fluents=infer_static_fluents(domain)
)
    costs = Dict{Term,Float64}()
    cost_fluents = PDDL.constituents(metric, domain)
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
