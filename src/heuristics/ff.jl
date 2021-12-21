## FastForward (FF) delete-relaxation heuristic ##
export FFHeuristic

"FastForward (FF) delete-relaxation heuristic."
mutable struct FFHeuristic <: Heuristic
    graph::PlanningGraph # Precomputed planning graph
    goal_idxs::Set{Int} # Precomputed list of goal indices
    pre_key::Tuple{UInt64,UInt64} # Precomputation hash
    FFHeuristic() = new()
end

Base.hash(heuristic::FFHeuristic, h::UInt) = hash(FFHeuristic, h)

function precompute!(h::FFHeuristic,
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

function is_precomputed(h::FFHeuristic,
                        domain::Domain, state::State, spec::Specification)
    return (isdefined(h, :graph) &&
            objectid(domain) == h.pre_key[1] &&
            objectid(spec) == h.pre_key[2])
end

function compute(h::FFHeuristic,
                 domain::Domain, state::State, spec::Specification)
    # Precompute if necessary
    if !is_precomputed(h, domain, state, spec)
        precompute!(h, domain, state, spec)
    end
    # Compute achievers to each condition node of the relaxed planning graph
    _, achievers = relaxed_graph_search(domain, state, spec,
                                        maximum, h.graph, h.goal_idxs)
    # Extract cost of relaxed plan via backward chaining
    cost = 0.0f0
    queue = collect(h.goal_idxs)
    while length(queue) > 0
        cond_idx = popfirst!(queue)
        act_idx = achievers[cond_idx]
        act_idx == -1 && continue # Skip conditions achieved from the start
        if has_action_cost(spec)
            act = h.graph.actions[act_idx].term
            cost += get_action_cost(spec, act)
        else
            cost += 1
        end
        append!(queue, h.graph.act_parents[act_idx])
    end
    # TODO: Store helpful actions
    # Return cost of relaxed plan as heuristic estimate
    return cost
end
