## HSP family of heuristics ##
export HSPHeuristic, HAdd, HMax
export HSPRHeuristic, HAddR, HMaxR

"HSP family of relaxation heuristics."
mutable struct HSPHeuristic{F <: Function} <: Heuristic
    op::F # Aggregator (e.g. maximum, sum) for fact costs
    graph::PlanningGraph # Precomputed planning graph
    HSPHeuristic{F}() where {F <: Function} = new{F}(F.instance)
    HSPHeuristic{F}(graph) where {F <: Function} = new{F}(F.instance, graph)
    HSPHeuristic(op::F) where {F <: Function} = new{F}(op)
    HSPHeuristic(op::F, graph) where {F <: Function} = new{F}(op, graph)
end

"HSP heuristic where a fact's cost is the maximum cost of its dependencies."
const HMax = HSPHeuristic{typeof(maximum)}

"HSP heuristic where a fact's cost is the summed cost of its dependencies."
const HAdd = HSPHeuristic{typeof(sum)}

function Base.show(io::IO, h::HSPHeuristic)
    is_precomputed_str = "precomputed=$(is_precomputed(h))"
    print(io, summary(h), "(", h.op, ", ", is_precomputed_str, ")")
end

Base.hash(heuristic::HSPHeuristic, h::UInt) =
    hash(heuristic.op, hash(HSPHeuristic, h))

is_precomputed(h::HSPHeuristic) = isdefined(h, :graph)

function precompute!(h::HSPHeuristic,
                     domain::Domain, state::State, spec::Specification)
    # Build planning graph and find goal condition indices
    goal = Compound(:and, get_goal_terms(spec))
    h.graph = build_planning_graph(domain, state, goal)
    return h
end

function compute(h::HSPHeuristic,
                 domain::Domain, state::State, spec::Specification)
    # Compute relaxed costs to each condition node of the planning graph
    _, _, _, goal_cost = relaxed_pgraph_search(domain, state, spec, h.op, h.graph)
    # Return goal cost (may be infinite if unreachable)
    return goal_cost
end

"HSPr family of delete-relaxation heuristics for regression search."
mutable struct HSPRHeuristic{F <: Function} <: Heuristic
    op::F
    costs::Dict{Term,Float64} # Est. cost of reaching each fact from goal
    HSPRHeuristic{F}() where {F <: Function} = new{F}(F.instance)
    HSPRHeuristic(op::F) where {F <: Function} = new{F}(op)
end

"HSPr heuristic where a fact's cost is the maximum cost of its dependencies."
const HMaxR = HSPRHeuristic{typeof(maximum)}

"HSPr heuristic where a fact's cost is the summed cost of its dependencies."
const HAddR = HSPRHeuristic{typeof(sum)}

function Base.show(io::IO, h::HSPRHeuristic)
    is_precomputed_str = "precomputed=$(is_precomputed(h))"
    print(io, summary(h), "(", h.op, ", ", is_precomputed_str, ")")
end

Base.hash(heuristic::HSPRHeuristic, h::UInt) =
    hash(heuristic.op, hash(HSPRHeuristic, h))

is_precomputed(h::HSPRHeuristic) = isdefined(h, :costs)

function precompute!(h::HSPRHeuristic,
                     domain::Domain, state::State, spec::Specification)
    # Construct and compute fact costs from planning graph
    graph = build_planning_graph(domain, state)
    costs, _, _, _ = relaxed_pgraph_search(domain, state, spec, h.op, graph)
    # Convert costs to dictionary for fast look-up
    h.costs = Dict{Term,Float64}(c => v for (c, v) in zip(graph.conditions, costs))
    return h
end

function compute(h::HSPRHeuristic,
                 domain::Domain, state::State, spec::Specification)
    # Compute cost of achieving all facts in current state
    facts = PDDL.get_facts(state)
    # TODO: Handle negative literals
    if length(facts) == 0 return 0.0 end
    return h.op(get(h.costs, f, 0) for f in facts)
end
