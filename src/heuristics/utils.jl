# Utilities and data structures for heuristic computation

"Planning graph used by relaxation-based heuristics."
struct PlanningGraph
    actions::Vector{GroundAction} # All ground actions
    act_parents::Vector{Vector{Int}} # Parent conditions of each action
    act_children::Vector{Vector{Int}} # Child conditions of each action
    conditions::Vector{Term} # All ground preconditions / goal conditions
    cond_children::Vector{Vector{Int}} # Child actions of each condition
end

"""
    build_planning_graph(domain, state[, goal_conds])

Construct planning graph for a `domain` grounded in a `state`, with optional
`goal_conds` that will be included as condition nodes.
"""
function build_planning_graph(domain::Domain, state::State,
                              goal_conds=Term[])
    # Generate list of ground actions, flattening conditional actions
    actions = GroundAction[]
    for act in groundactions(domain, state)
        if act.effect isa PDDL.GenericDiff
            push!(actions, act)
        else # Handle conditional effects
            append!(actions, PDDL.flatten_conditions(act))
        end
    end
    # Extract conditions and effects of ground actions, storing their
    cond_map = Dict{Term,Set{Int}}() # Map conditions to action indices
    effect_map = Dict{Term,Set{Int}}() # Map effects to action indices
    for (i, act) in enumerate(actions)
        preconds = isempty(act.preconds) ? Term[Const(true)] : act.preconds
        for cond in preconds # Preconditions
            idxs = get!(cond_map, cond, Set{Int}())
            push!(idxs, i)
        end
        for eff in act.effect.add # Add effects
            idxs = get!(effect_map, eff, Set{Int}())
            push!(idxs, i)
        end
        for eff in act.effect.del # Delete effects
            idxs = get!(effect_map, Compound(:not, [eff]), Set{Int}())
            push!(idxs, i)
        end
        for (term, _) in act.effect.ops # Assignment effects
            idxs = get!(effect_map, term, Set{Int}())
            push!(idxs, i)
        end
    end
    # Insert goal conditions
    for g in goal_conds get!(cond_map, g, Set{Int}()) end
    # Flatten map from conditions to child indices
    cond_children = [sort!(collect(is)) for is in values(cond_map)]
    conditions = collect(keys(cond_map))
    # Determine parent and child conditions of each action
    act_parents = [Int[] for _ in 1:length(actions)]
    act_children = [Int[] for _ in 1:length(actions)]
    for (i, cond) in enumerate(conditions)
        # Collect parent conditions
        idxs = get(cond_map, cond, Set{Int}())
        push!.(act_parents[collect(idxs)], i)
        # Collect child conditions
        if cond.name == :not || PDDL.is_pred(cond, domain) # Handle literals
            idxs = get(effect_map, cond, Set{Int}())
        elseif cond.name == :or # Handle disjunctions
            idxs = reduce(union, (get(effect_map, a, Set{Int}()) for a in cond.args))
        elseif cond.name == true || cond.name == false # Handle constants
            idxs = get(effect_map, cond, Set{Int}())
        else # Handle functional terms
            terms = PDDL.constituents(cond, domain)
            idxs = reduce(union, (get(effect_map, t, Set{Int}()) for t in terms))
        end
        push!.(act_children[collect(idxs)], i)
    end
    act_children = unique!.(sort!.(act_children))
    # Construct and return graph
    return PlanningGraph(actions, act_parents, act_children,
                         conditions, cond_children)
end

"Compute relaxed costs and paths to each fact node of a planning graph."
function relaxed_graph_search(
    domain::Domain, state::State, spec::Specification,
    accum_op::Function, graph::PlanningGraph, goal_idxs=nothing
)
    # Initialize fact costs, counters,  etc.
    n_conds = length(graph.conditions)
    costs = fill(Inf16, n_conds) # Fact costs
    achievers = fill(-1, n_conds) # Fact achievers
    counters = [length(a.preconds) for a in graph.actions] # Action counters

    # Set up initial facts and priority queue
    init_idxs = _get_init_idxs(graph, domain, state)
    costs[init_idxs] .= 0
    queue = PriorityQueue{Int,Float16}(i => 0 for i in init_idxs)

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
        # Iterate over child actions
        for act_idx in graph.cond_children[cond_idx]
            # Decrease counter for unachieved actions
            counters[act_idx] -= 1
            counters[act_idx] > 0 && continue
            # Compute path cost of achieved action
            act_parents = graph.act_parents[act_idx]
            path_cost = accum_op(costs[p] for p in act_parents)
            act_cost = has_action_cost(spec) ?
                get_action_cost(spec, domain, graph.actions[act_idx].term) : 1
            next_cost = path_cost + act_cost
            # Place child conditions on queue
            act_children = graph.act_children[act_idx]
            for c_idx in act_children
                if next_cost > costs[c_idx] continue end
                costs[c_idx] = next_cost # Store new cost
                achievers[c_idx] = act_idx # Store new cheapest achiever
                if !(c_idx in keys(queue)) # Enqueue new conditions
                    enqueue!(queue, c_idx, next_cost)
                else # Reduce the cost of those in queue
                    queue[c_idx] = next_cost
                end
                delete!(unreached, c_idx) # Removed any reached goals
            end
        end
    end

    # Return fact costs and achievers
    return costs, achievers
end

"Returns planning graph indices for initial facts."
function _get_init_idxs(graph::PlanningGraph,
                       domain::Domain, state::State)
    return [i for (i, cond) in enumerate(graph.conditions)
            if satisfy(domain, state, cond)]
end

function _get_init_idxs(graph::PlanningGraph,
                        domain::Domain, state::GenericState)
    init_facts = PDDL.get_facts(state)
    pos_idxs = findall(c -> c in init_facts || c.name == true,
                       graph.conditions)
    neg_idxs = findall(c -> c.name == :not && !(c.args[1] in init_facts),
                       graph.conditions)
    init_idxs = append!(pos_idxs, neg_idxs)
    if !isempty(PDDL.get_functions(domain))
        func_idxs = findall(graph.conditions) do c
            return ((PDDL.is_func(c, domain) || PDDL.is_global_func(c)) &&
                    satisfy(domain, state, c))
        end
        append!(init_idxs, func_idxs)
    end
    return init_idxs
end
