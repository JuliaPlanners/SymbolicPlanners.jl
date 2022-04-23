## HSP family of heuristics ##
export HSPHeuristic, HAdd, HMax
export HSPRHeuristic, HAddR, HMaxR

"HSP family of relaxation heuristics."
mutable struct HSPHeuristic <: Heuristic
    op::Function # Aggregator (e.g. maximum, sum) for fact costs
    graph::PlanningGraph # Precomputed planning graph
    goal_idxs::Set{Int} # Precomputed list of goal indices
    HSPHeuristic(op) = new(op)
    HSPHeuristic(op, graph, goal_idxs) = new(op, graph, goal_idxs)
end

Base.hash(heuristic::HSPHeuristic, h::UInt) =
    hash(heuristic.op, hash(HSPHeuristic, h))

is_precomputed(h::HSPHeuristic) =
    isdefined(h, :graph) && isdefined(h, :goal_idxs)

function precompute!(h::HSPHeuristic,
                     domain::Domain, state::State, spec::Specification)
    # Build planning graph and find goal condition indices
    goal_conds = PDDL.to_cnf_clauses(get_goal_terms(spec))
    h.graph = build_planning_graph(domain, state, goal_conds)
    h.goal_idxs = Set(findall(c -> c in goal_conds, h.graph.conditions))
    return h
end

function compute(h::HSPHeuristic,
                 domain::Domain, state::State, spec::Specification)
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
    HSPRHeuristic(op) = new(op)
end

Base.hash(heuristic::HSPRHeuristic, h::UInt) =
    hash(heuristic.op, hash(HSPRHeuristic, h))

is_precomputed(h::HSPRHeuristic) = isdefined(h, :costs)

function precompute!(h::HSPRHeuristic,
                     domain::Domain, state::State, spec::Specification)
    # Construct and compute fact costs from planning graph
    graph = build_planning_graph(domain, state)
    costs, _ = relaxed_graph_search(domain, state, spec, h.op, graph)
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

"HSPr heuristic where a fact's cost is the maximum cost of its dependencies."
HMaxR(args...) = HSPRHeuristic(maximum, args...)

"HSPr heuristic where a fact's cost is the summed cost of its dependencies."
HAddR(args...) = HSPRHeuristic(sum, args...)
