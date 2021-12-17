## HSP family of heuristics ##
export HSPHeuristic, HAdd, HMax
export HSPRHeuristic, HAddR, HMaxR
export FunctionalHSP, FunctionalHAdd, FunctionalHMax

"HSP family of relaxation heuristics."
mutable struct HSPHeuristic <: Heuristic
    op::Function # Aggregator (e.g. maximum, sum) for fact costs
    graph::PlanningGraph # Precomputed planning graph
    goal_idxs::Set{Int} # Precomputed list of goal indices
    pre_key::Tuple{UInt64,UInt64} # Precomputation hash
    HSPHeuristic(op) = new(op)
end

Base.hash(heuristic::HSPHeuristic, h::UInt) =
    hash(heuristic.op, hash(HSPHeuristic, h))

function precompute!(h::HSPHeuristic,
                     domain::Domain, state::State, spec::Specification)
    # Check if cache has already been computed
    if is_precomputed(h, domain, state, spec) return h end
    # Precomputed data is unique to each domain and specification
    h.pre_key = (objectid(domain), objectid(spec))
    # Build planning graph and find goal condition indices
    goal_conds = PDDL.to_cnf_clauses(get_goal_terms(spec))
    h.graph = build_planning_graph(domain, state, goal_conds)
    h.goal_idxs = Set(findall(c -> c in goal_conds, h.graph.conditions))
    return h
end

function is_precomputed(h::HSPHeuristic,
                        domain::Domain, state::State, spec::Specification)
    return (isdefined(h, :graph) &&
            objectid(domain) == h.pre_key[1] &&
            objectid(spec) == h.pre_key[2])
end

function compute(h::HSPHeuristic,
                 domain::Domain, state::State, spec::Specification)
    # Precompute if necessary
    if !is_precomputed(h, domain, state, spec)
        precompute!(h, domain, state, spec)
    end
    # Compute relaxed costs to each condition node of the planning graph
    costs, _ = relaxed_graph_search(domain, state, spec,
                                    h.op, h.graph, h.goal_idxs)
    # Return goal cost (may be infinite if unreachable)
    goal_cost = h.op(costs[g] for g in h.goal_idxs)
    return goal_cost
end

"HSP heuristic where a fact's cost is the maximum cost of its dependencies."
HMax(args...) = HSPHeuristic(maximum, args...)

"HSP heuristic where a fact's cost is the summed cost of its dependencies."
HAdd(args...) = HSPHeuristic(sum, args...)

"HSPr family of delete-relaxation heuristics for regression search."
mutable struct HSPRHeuristic <: Heuristic
    op::Function
    costs::Dict{Term,Float64} # Est. cost of reaching each fact from goal
    pre_key::UInt64 # Precomputation key
    HSPRHeuristic(op) = new(op)
end

Base.hash(heuristic::HSPRHeuristic, h::UInt) =
    hash(heuristic.op, hash(HSPRHeuristic, h))

function precompute!(h::HSPRHeuristic,
                     domain::Domain, state::State, spec::Specification)
    # Check if data has already been computed
    if is_precomputed(h, domain, state, spec) return h end
    # Precomputed data is tied to domain
    h.pre_key = objectid(domain)
    # Construct and compute fact costs from planning graph
    graph = build_planning_graph(domain, state)
    costs, _ = relaxed_graph_search(domain, state, spec, h.op, graph)
    # Convert costs to dictionary for fast look-up
    h.costs = Dict{Term,Float64}(c => v for (c, v) in zip(graph.conditions, costs))
    return h
end

function is_precomputed(h::HSPRHeuristic,
                        domain::Domain, state::State, spec::Specification)
    return isdefined(h, :costs) && objectid(domain) == h.pre_key
end

function compute(h::HSPRHeuristic,
                 domain::Domain, state::State, spec::Specification)
    # Precompute if necessary
    if !is_precomputed(h, domain, state, spec)
        precompute!(h, domain, state, spec)
    end
    # Compute cost of achieving all facts in current state
    facts = PDDL.get_facts(state)
    # TODO: Handle negative literals
    if length(facts) == 0 return 0.0 end
    return h.op(get(h.costs, f, 0) for f in facts)
end

"HSPr heuristic where a fact's cost is the maximum cost of its dependencies."
HMaxR(args...) = HSPRHeuristic(maximum, args...)

"HSPr heuristic where a fact's cost is the summed cost of its dependencies."
HAddR(args...) = HSPRHeuristic(sum, args...)

"Generalized HSP heuristic which handles functional conditions."
mutable struct FunctionalHSP <: Heuristic
    op::Function # Aggregator (e.g. maximum, sum) for fact costs
    abstractions::Dict{Symbol,Type} # Abstractions to use
    max_repeats::Int # Maximum number of action repeats
    graph::PlanningGraph # Precomputed planning graph
    goal_idxs::Set{Int} # Precomputed list of goal indices
    pre_key::Tuple{UInt64,UInt64} # Precomputation hash
    FunctionalHSP(op, abstractions, max_repeats) =
        new(op, abstractions, max_repeats)
end

FunctionalHSP(op; abstractions=PDDL.DEFAULT_ABSTRACTIONS, max_repeats=10) =
    FunctionalHSP(op, abstractions, max_repeats)

Base.hash(heuristic::FunctionalHSP, h::UInt) =
    hash(heuristic.op, hash(heuristic.abstractions, hash(FunctionalHSP, h)))

function precompute!(h::FunctionalHSP,
                     domain::Domain, state::State, spec::Specification)
    # Check if cache has already been computed
    if is_precomputed(h, domain, state, spec) return h end
    # Precomputed data is unique to each domain and specification
    h.pre_key = (objectid(domain), objectid(spec))
    # Build planning graph and find goal condition indices
    goal_conds = PDDL.to_cnf_clauses(get_goal_terms(spec))
    h.graph = build_planning_graph(domain, state, goal_conds)
    h.goal_idxs = Set(findall(c -> c in goal_conds, h.graph.conditions))
    return h
end

function is_precomputed(h::FunctionalHSP,
                        domain::Domain, state::State, spec::Specification)
    return (isdefined(h, :graph) &&
            objectid(domain) == h.pre_key[1] &&
            objectid(spec) == h.pre_key[2])
end

function compute(h::FunctionalHSP,
                 domain::Domain, state::State, spec::Specification)
    # Precompute if necessary
    if !is_precomputed(h, domain, state, spec)
        precompute!(h, domain, state, spec)
    end
    # Compute relaxed costs to each condition node of the planning graph
    costs, _ = relaxed_functional_graph_search(
        domain, state, spec,
        h.op, h.graph, h.abstractions,
        h.max_repeats, h.goal_idxs
    )
    # Return goal cost (may be infinite if unreachable)
    goal_cost = h.op(costs[g] for g in h.goal_idxs)
    return goal_cost
end

"Generalized [`HMax`](@ref) heuristic that supports functional conditions."
FunctionalHMax(; options...) = FunctionalHSP(maximum; options...)

"Generalized [`HAdd`](@ref) heuristic that supports functional conditions."
FunctionalHAdd(; options...) = FunctionalHSP(sum; options...)
