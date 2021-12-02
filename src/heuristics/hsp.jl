## HSP family of heuristics ##
export HSPHeuristic, HAdd, HMax
export HSPRHeuristic, HAddR, HMaxR

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
    costs = relaxed_cost_search(domain, state, spec, h.op, h.graph, h.goal_idxs)

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
    costs = relaxed_cost_search(domain, state, spec, h.op, graph)

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

"Compute relaxed costs to each fact node of a planning graph."
function relaxed_cost_search(
    domain::Domain, state::State, spec::Specification,
    accum_op::Function, graph::PlanningGraph, goal_idxs=nothing
)
    # Initialize fact costs, counters,  etc.
    costs = fill(Inf, length(graph.conditions)) # Fact costs
    expanded = falses(length(graph.conditions)) # Whether facts are expanded
    counters = [length(a.preconds) for a in graph.actions] # Action counters

    # Set up initial facts and priority queue
    init_facts = Set{Term}(PDDL.get_facts(state))
    push!(init_facts, Const(true))
    pos_idxs = findall(c -> c in init_facts, graph.conditions)
    neg_idxs = findall(c -> c.name == :not && !(c.args[1] in init_facts),
                       graph.conditions)
    init_idxs = append!(pos_idxs, neg_idxs)
    costs[init_idxs] .= 0
    queue = PriorityQueue(i => costs[i] for i in init_idxs)

    # Check if any goal conditions are already reached
    if goal_idxs !== nothing
        for g in goal_idxs
            goal_cond = graph.conditions[g]
            costs[g] == Inf && !satisfy(domain, state, goal_cond) && continue
            costs[g] = 0
        end
        unreached = copy(goal_idxs)
    else
        unreached = Set{Int}(-1) # Set with dummy unreachable node
    end

    # Perform Djikstra / uniform-cost search until goals are reached
    while !isempty(queue) && !isempty(unreached)
        # Dequeue lowest cost fact/condition
        cond_idx = dequeue!(queue)
        expanded[cond_idx] = true
        # Iterate over child actions
        for act_idx in graph.cond_children[cond_idx]
            # Decrease counter for unachieved actions
            counters[act_idx] -= 1
            counters[act_idx] > 0 && continue
            # Compute path cost of achieved action
            act_parents = graph.act_parents[act_idx]
            path_cost = accum_op(costs[p] for p in act_parents)
            act_cost = 1 # TODO: Support variable action costs
            next_cost = path_cost + act_cost
            # Place child conditions on queue
            act_children = graph.act_children[act_idx]
            for c_idx in act_children
                # TODO: Handle functional conditions
                if next_cost > costs[c_idx] continue end
                costs[c_idx] = next_cost
                if !(c_idx in keys(queue)) # Enqueue new conditions
                    enqueue!(queue, c_idx, next_cost)
                else # Reduce the cost of those in queue
                    queue[c_idx] = next_cost
                end
                delete!(unreached, c_idx) # Removed any reached goals
            end
        end
    end

    # Return fact costs
    return costs
end
