## HSP family of heuristics ##
export HSPHeuristic, HAdd, HMax
export HSPRHeuristic, HAddR, HMaxR

"HSP family of relaxation heuristics."
mutable struct HSPHeuristic{F <: Function} <: Heuristic
    op::F # Aggregator (e.g. maximum, sum) for fact costs
    dynamic_goal::Bool # Flag whether goal-relevant information is dynamic
    goal_hash::Union{Nothing,UInt} # Hash of most recently pre-computed goal
    statics::Vector{Symbol} # Static domain fluents
    graph::PlanningGraph # Precomputed planning graph
    HSPHeuristic{F}() where {F <: Function} = new{F}(F.instance)
    HSPHeuristic(op::F) where {F <: Function} = new{F}(op)
end

"HSP heuristic where a fact's cost is the maximum cost of its dependencies."
const HMax = HSPHeuristic{typeof(maximum)}

"HSP heuristic where a fact's cost is the summed cost of its dependencies."
const HAdd = HSPHeuristic{typeof(sum)}

function Base.show(io::IO, h::HSPHeuristic)
    is_precomputed_str = "precomputed=$(is_precomputed(h))"
    print(io, summary(h), "(", h.op, ", ", is_precomputed_str, ")")
end

is_precomputed(h::HSPHeuristic) = isdefined(h, :graph)

function precompute!(h::HSPHeuristic,
                     domain::Domain, state::State)
    # If goal specification is not provided, assume dynamic goal
    h.dynamic_goal = true
    h.goal_hash = nothing
    h.statics = infer_static_fluents(domain)
    h.graph = build_planning_graph(domain, state; statics=h.statics)
    return h
end

function precompute!(h::HSPHeuristic,
                     domain::Domain, state::State, spec::Specification)
    # If goal specification is provided, assume non-dynamic goal
    h.dynamic_goal = false
    h.goal_hash = hash(spec)
    h.statics = infer_static_fluents(domain)
    h.graph = build_planning_graph(domain, state, spec; statics=h.statics)
    return h
end

function compute(h::HSPHeuristic,
                 domain::Domain, state::State, spec::Specification)
    # If necessary, update planning graph with new goal
    if h.dynamic_goal && hash(spec) != h.goal_hash
        h.graph = update_pgraph_goal!(h.graph, domain, state, spec;
                                      statics=h.statics)
        h.goal_hash = hash(spec)
    end
    # Compute relaxed costs to goal nodes of the planning graph
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
