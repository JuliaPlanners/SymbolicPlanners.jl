# Utilities and data structures for heuristic computation

"Planning graph used by relaxation-based heuristics."
struct PlanningGraph
    actions::Vector{GroundAction} # All ground actions
    act_parents::Vector{Vector{Int}} # Parent conditions of each action
    act_children::Vector{Vector{Int}} # Child conditions of each action
    conditions::Vector{Term} # All ground preconditions / goal conditions
    cond_children::Vector{Vector{Int}} # Child actions of each condition
    is_func::BitVector # Whether condition is a function
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
    is_func = falses(length(conditions))
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
            is_func[i] = true
            terms = PDDL.constituents(cond, domain)
            idxs = reduce(union, (get(effect_map, t, Set{Int}()) for t in terms))
        end
        push!.(act_children[collect(idxs)], i)
    end
    act_children = unique!.(sort!.(act_children))
    # Construct and return graph
    return PlanningGraph(actions, act_parents, act_children,
                         conditions, cond_children, is_func)
end

"Compute relaxed costs and paths to each fact node of a planning graph."
function relaxed_graph_search(
    domain::Domain, state::State, spec::Specification,
    accum_op::Function, graph::PlanningGraph, goal_idxs=nothing
)
    # Initialize fact costs, counters,  etc.
    costs = fill(Inf, length(graph.conditions)) # Fact costs
    achievers = fill(-1, length(graph.conditions)) # Fact achievers
    counters = [length(a.preconds) for a in graph.actions] # Action counters

    # Set up initial facts and priority queue
    init_idxs = _get_init_idxs(graph, domain, state)
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

"Relaxed graph search with functional conditions."
function relaxed_functional_graph_search(
    domain::Domain, state::State, spec::Specification,
    accum_op::Function, graph::PlanningGraph,
    abstractions::Dict, max_repeats::Int, goal_idxs=nothing
)
    # Initialize fact costs, counters,  etc.
    costs = fill(Inf, length(graph.conditions)) # Fact costs
    dists = accum_op === maximum ? costs : fill(-1, length(graph.conditions))
    achievers = fill(-1, length(graph.conditions)) # Fact achievers
    counters = [length(a.preconds) for a in graph.actions] # Action counters
    repeats = fill(0, length(graph.actions)) # Action repetitinos

    # Set up initial facts and priority queue
    init_idxs = _get_init_idxs(graph, domain, state)
    costs[init_idxs] .= 0
    if (accum_op !== maximum) dists[init_idxs] .= 0 end
    queue = PriorityQueue(i => dists[i] for i in init_idxs)

    # Check if any goal conditions are already reached
    if goal_idxs !== nothing
        for g in goal_idxs
            goal_cond = graph.conditions[g]
            costs[g] == Inf && !satisfy(domain, state, goal_cond) && continue
            costs[g] = 0
            if (accum_op !== maximum) dists[g] = 0 end
        end
        unreached = copy(goal_idxs)
    else
        unreached = Set{Int}(-1) # Set with dummy unreachable node
    end

    # Construct abstracted domain and state
    domain = domain isa GenericDomain ? domain : PDDL.get_source(domain)
    state = state isa GenericState ? state : GenericState(state)
    absdom = abstracted(domain; abstractions=abstractions)
    abstate = abstractstate(absdom, state)

    # Define action expansion subroutine
    function expand_act!(act_idx)
        # Increment repetition counter
        repeats[act_idx] += 1
        is_max_rep = repeats[act_idx] >= max_repeats

        # Compute path cost of achieved action
        act_parents = graph.act_parents[act_idx]
        path_cost = accum_op(costs[p] for p in act_parents)
        act_cost = has_action_cost(spec) ?
            get_action_cost(spec, domain, graph.actions[act_idx].term) : 1
        next_cost = path_cost + act_cost * repeats[act_idx]

        # Compute path distance
        path_dist = accum_op === maximum ?
            path_cost : maximum(dists[p] for p in act_parents)
        next_dist = accum_op === maximum ?
            next_cost : path_dist + repeats[act_idx]

        # Widen abstract state with functional effects of action
        diff = graph.actions[act_idx].effect
        vals = [evaluate(absdom, abstate, v) for v in values(diff.ops)]
        for (term, val) in zip(keys(diff.ops), vals)
            widened = PDDL.widen(abstate[term], val)
            abstate[term] = widened
        end

        # Place child conditions on queue
        act_children = graph.act_children[act_idx]
        requeue_act = false
        for c_idx in act_children
            # Skip conditions which are already reached more cheaply
            if next_cost > costs[c_idx] continue end
            # Handle functional conditions
            if (!is_max_rep && graph.is_func[c_idx] &&
                !satisfy(absdom, abstate, graph.conditions[c_idx]))
                # Requeue action if functional condition not satisfied
                requeue_act = true
                continue
                # Otherwise assume condition satisfied (incl. after max reps)
            end
            # Store new cost, distance and achiever
            costs[c_idx] = next_cost
            if (accum_op !== maximum) dists[c_idx] = next_dist end
            achievers[c_idx] = act_idx # Store new cheapest achiever
            if !(c_idx in keys(queue)) # Enqueue new conditions
                enqueue!(queue, c_idx, next_dist)
            else # Reduce the cost of those in queue
                queue[c_idx] = next_dist
            end
            delete!(unreached, c_idx) # Removed any reached goals
        end

        # Requeue action if necessary
        if requeue_act
            if !(-act_idx in keys(queue))
                enqueue!(queue, -act_idx, next_dist)
            else
                queue[-act_idx] = next_dist
            end
        end
    end

    # Perform Djikstra / uniform-cost search until goals are reached
    while !isempty(queue) && !isempty(unreached)
        # Dequeue lowest cost fact/condition
        cond_idx = dequeue!(queue)
        if cond_idx < 0 # Convention: negative indices are repeated actions
            act_idx = -cond_idx
            expand_act!(act_idx)
        else # Iterate over child actions
            for act_idx in graph.cond_children[cond_idx]
                # Decrease counter for unachieved actions
                counters[act_idx] -= 1
                counters[act_idx] != 0 && continue
                expand_act!(act_idx)
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
