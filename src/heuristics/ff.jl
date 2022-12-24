## FastForward (FF) delete-relaxation heuristic ##
export FFHeuristic

"FastForward (FF) delete-relaxation heuristic."
mutable struct FFHeuristic <: Heuristic
    dynamic_goal::Bool # Flag whether goal-relevant information is dynamic
    goal_hash::Union{Nothing,UInt} # Hash of most recently pre-computed goal
    statics::Vector{Symbol} # Static domain fluents 
    graph::PlanningGraph # Precomputed planning graph
    FFHeuristic() = new()
end

function Base.show(io::IO, h::FFHeuristic)
    print(io, summary(h), "(precomputed=$(is_precomputed(h)))")
end

is_precomputed(h::FFHeuristic) = isdefined(h, :graph)

function precompute!(h::FFHeuristic,
                     domain::Domain, state::State)
    # If goal specification is not provided, assume dynamic goal
    h.dynamic_goal = true
    h.goal_hash = nothing
    h.statics = infer_static_fluents(domain)
    h.graph = build_planning_graph(domain, state; statics=h.statics)
    return h
end

function precompute!(h::FFHeuristic,
                     domain::Domain, state::State, spec::Specification)
    # If goal specification is provided, assume non-dynamic goal
    h.dynamic_goal = false
    h.goal_hash = hash(spec)
    h.statics = infer_static_fluents(domain)
    h.graph = build_planning_graph(domain, state, spec; statics=h.statics)
    return h
end

function compute(h::FFHeuristic,
                 domain::Domain, state::State, spec::Specification)
    # If necessary, update planning graph with new goal
    if h.dynamic_goal && hash(spec) != h.goal_hash
        h.graph = update_pgraph_goal!(h.graph, domain, state, spec;
                                      statics=h.statics)
        h.goal_hash = hash(spec)
    end
    # Compute achievers to each condition node of the relaxed planning graph
    costs, achievers, goal_idx, _ =
        relaxed_pgraph_search(domain, state, spec, maximum, h.graph)
    # Return infinity if goal is not reached
    if isnothing(goal_idx) return Inf32 end
    # Initialize queue
    queue = Int[]
    goal_parents = h.graph.act_parents[goal_idx]
    ff_add_parent_conds_to_queue!(queue, goal_parents, costs)
    # Extract cost of relaxed plan via backward chaining
    plan_cost = 0.0f0
    act_idxs = Int[]
    while !isempty(queue)
        cond_idx = popfirst!(queue)
        act_idx = achievers[cond_idx]
        act_idx == -1 && continue # Skip conditions achieved from the start
        act_idx in act_idxs && continue # Skip actions that already appeared
        push!(act_idxs, act_idx)
        if act_idx > h.graph.n_axioms # Add cost to plan for non-axioms
            plan_cost += has_action_cost(spec) ?
                get_action_cost(spec, h.graph.actions[act_idx].term) : 1
        end
        # Push supporting condition indices onto queue
        act_parents = h.graph.act_parents[act_idx]
        ff_add_parent_conds_to_queue!(queue, act_parents, costs)
    end
    # TODO: Store helpful actions
    # Return cost of relaxed plan as heuristic estimate
    return plan_cost
end

"Adds parent conditions of an action to the queue."
function ff_add_parent_conds_to_queue!(queue, act_parents, costs)
    for precond_parents in act_parents
        if length(precond_parents) == 1
            push!(queue, precond_parents[1])
        else
            _, min_idx = findmin(costs[precond_parents])
            min_cond_idx = precond_parents[min_idx]
            push!(queue, min_cond_idx)
        end
    end
    return queue
end
