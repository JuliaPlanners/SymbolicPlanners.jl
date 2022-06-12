## FastForward (FF) delete-relaxation heuristic ##
export FFHeuristic

"FastForward (FF) delete-relaxation heuristic."
mutable struct FFHeuristic <: Heuristic
    graph::PlanningGraph # Precomputed planning graph
    goal_idxs::Set{Int} # Precomputed list of goal indices
    FFHeuristic() = new()
    FFHeuristic(graph, goal_idxs) = new(graph, goal_idxs)
end

Base.hash(heuristic::FFHeuristic, h::UInt) = hash(FFHeuristic, h)

is_precomputed(h::FFHeuristic) =
    isdefined(h, :graph) && isdefined(h, :goal_idxs)

function precompute!(h::FFHeuristic,
                     domain::Domain, state::State, spec::Specification)
    # Build planning graph and find goal condition indices
    goal_conds = PDDL.to_cnf_clauses(get_goal_terms(spec))
    h.graph = build_planning_graph(domain, state, goal_conds)
    h.goal_idxs = Set(findall(c -> c in goal_conds, h.graph.conditions))
    return h
end

function compute(h::FFHeuristic,
                 domain::Domain, state::State, spec::Specification)
    # Compute achievers to each condition node of the relaxed planning graph
    _, achievers = relaxed_graph_search(domain, state, spec,
                                        maximum, h.graph, h.goal_idxs)
    # Extract cost of relaxed plan via backward chaining
    cost = 0.0f0
    queue = collect(h.goal_idxs)
    act_idxs = Int[]
    while !isempty(queue)
        cond_idx = popfirst!(queue)
        act_idx = achievers[cond_idx]
        act_idx == -1 && continue # Skip conditions achieved from the start
        act_idx in act_idxs && continue # Skip actions that already appeared
        push!(act_idxs, act_idx)
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
