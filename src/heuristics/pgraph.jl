# Planning graph utilities for relaxed heuristic computation

"Planning graph used by relaxation-based heuristics."
struct PlanningGraph
    actions::Vector{GroundAction} # All ground actions
    act_parents::Vector{Vector{Int}} # Parent conditions of each action
    act_children::Vector{Vector{Int}} # Child conditions of each action
    n_axioms::Int # Number of ground actions converted from axioms
    conditions::Vector{Term} # All ground preconditions / goal conditions
    cond_children::Vector{Vector{Tuple{Int,Int}}} # Child actions of each condition
    cond_derived::BitVector # Whether the conditions are derived
    cond_functional::BitVector # Whether the conditions involve functions
end

"""
    build_planning_graph(domain, state[, goal_conds])

Construct planning graph for a `domain` grounded in a `state`, with optional
`goal_conds` that will be included as condition nodes.
"""
function build_planning_graph(domain::Domain, state::State,
                              goal_conds=Term[])
    # Populate list of ground actions and converted axioms
    actions = GroundAction[]
    # Add axioms converted to ground actions
    for ax in groundaxioms(domain, state)
        if ax.effect isa PDDL.GenericDiff
            push!(actions, ax)
        else # Handle conditional effects
            append!(actions, PDDL.flatten_conditions(ax))
        end
    end
    n_axioms = length(actions)
    # Add ground actions, flattening conditional actions
    for act in groundactions(domain, state)
        # TODO: Eliminate redundant actions
        if act.effect isa PDDL.GenericDiff
            push!(actions, act)
        else # Handle conditional effects
            append!(actions, PDDL.flatten_conditions(act))
        end
    end
    # Extract conditions and effects of ground actions
    cond_map = Dict{Term,Vector{Tuple{Int,Int}}}() # Map conditions to action indices
    effect_map = Dict{Term,Vector{Int}}() # Map effects to action indices
    for (i, act) in enumerate(actions)
        preconds = isempty(act.preconds) ? Term[Const(true)] : act.preconds
        for (j, cond) in enumerate(preconds) # Preconditions
            if cond.name == :or # Handle disjunctions
                for c in cond.args
                    idxs = get!(Vector{Tuple{Int,Int}}, cond_map, c)
                    push!(idxs, (i, j)) # Map to jth condition of ith action
                end
            else
                idxs = get!(Vector{Tuple{Int,Int}}, cond_map, cond)
                push!(idxs, (i, j)) # Map to jth condition of ith action
            end
        end
        for eff in act.effect.add # Add effects
            idxs = get!(Vector{Int}, effect_map, eff)
            push!(idxs, i)
        end
        for eff in act.effect.del # Delete effects
            idxs = get!(Vector{Int}, effect_map, Compound(:not, Term[eff]))
            push!(idxs, i)
        end
        for (term, _) in act.effect.ops # Assignment effects
            idxs = get!(Vector{Int}, effect_map, term)
            push!(idxs, i)
        end
    end
    # Insert goal conditions
    for g in goal_conds get!(Vector{Tuple{Int,Int}}, cond_map, g) end
    # Flatten map from conditions to child indices
    cond_children = collect(values(cond_map))
    conditions = collect(keys(cond_map))
    # Determine parent and child conditions of each action
    act_parents = [Int[] for _ in 1:length(actions)]
    act_children = [Int[] for _ in 1:length(actions)]
    for (i, cond) in enumerate(conditions)
        # Collect parent conditions
        idxs = first.(get(Vector{Tuple{Int,Int}}, cond_map, cond))
        push!.(act_parents[idxs], i)
        # Collect child conditions
        if cond.name in (:not, true, false) || PDDL.is_pred(cond, domain)
            idxs = get(Vector{Int}, effect_map, cond) # Handle literals
        else # Handle functional terms
            terms = PDDL.constituents(cond, domain)
            idxs = reduce(union, (get(Vector{Int}, effect_map, t) for t in terms))
        end
        push!.(act_children[idxs], i)
    end
    act_children = unique!.(sort!.(act_children))
    # Determine if conditions are derived or functional
    cond_derived = isempty(PDDL.get_axioms(domain)) ?
        falses(length(conditions)) :
        broadcast(c -> PDDL.has_derived(c, domain), conditions)
    cond_functional = isempty(PDDL.get_functions(domain)) ?
        falses(length(conditions)) :
        broadcast(c -> PDDL.has_func(c, domain) ||
                       PDDL.has_global_func(c), conditions)
    # Construct and return graph
    return PlanningGraph(actions, act_parents, act_children,
                         n_axioms, conditions, cond_children,
                         cond_derived, cond_functional)
end

"Compute relaxed costs and paths to each fact node of a planning graph."
function relaxed_graph_search(
    domain::Domain, state::State, spec::Specification,
    accum_op::Function, graph::PlanningGraph, goal_idxs=nothing
)
    # Initialize fact costs, precondition flags,  etc.
    n_conds = length(graph.conditions)
    dists = fill(Inf32, n_conds) # Fact distances
    costs = fill(Inf32, n_conds) # Fact costs
    achievers = fill(-1, n_conds) # Fact achievers
    condflags = [(UInt(1) << length(a.preconds)) - 1 for a in graph.actions]

    # Set up initial facts and priority queue
    init_idxs = pgraph_init_idxs(graph, domain, state)
    dists[init_idxs] .= 0
    costs[init_idxs] .= 0
    queue = PriorityQueue{Int,Float32}(i => 0 for i in findall(init_idxs))

    # Check if any goal conditions are already reached
    if goal_idxs !== nothing
        for g in goal_idxs
            goal_cond = graph.conditions[g]
            if dists[g] === Inf32 && !satisfy(domain, state, goal_cond)
                continue
            end
            dists[g] = 0
            costs[g] = 0
        end
        unreached = copy(goal_idxs)
    else
        unreached = Set{Int}(-1) # Set with dummy unreachable node
    end

    # Perform Djikstra / uniform-cost search until goals are reached
    while !isempty(queue) && !isempty(unreached)
        # Dequeue nearest fact/condition
        cond_idx = dequeue!(queue)
        # Iterate over child actions
        for (act_idx, precond_idx) in graph.cond_children[cond_idx]
            # Skip actions with no children
            isempty(graph.act_children[act_idx]) && continue
            # Skip actions already achieved
            condflags[act_idx] === UInt(0) && continue
            # Set precondition flag for unachieved actions
            condflags[act_idx] &= ~(UInt(1) << (precond_idx-1))
            condflags[act_idx] != UInt(0) && continue
            # Compute path cost and distance of achieved action or axiom
            act_parents = graph.act_parents[act_idx]
            path_cost = accum_op(costs[p] for p in act_parents)
            is_axiom = act_idx <= graph.n_axioms
            if is_axiom # Axioms have zero cost
                next_cost = path_cost
                next_dist = dists[cond_idx]
            else # Lookup action cost if specified, default to one otherwise
                act_cost = has_action_cost(spec) ?
                    get_action_cost(spec, graph.actions[act_idx].term) : 1
                next_cost = path_cost + act_cost
                next_dist = accum_op === maximum && !has_action_cost(spec) ?
                    next_cost : dists[cond_idx] + 1
            end
            # Place child conditions on queue
            act_children = graph.act_children[act_idx]
            for c_idx in act_children
                less_dist = next_dist < dists[c_idx]
                less_cost = next_cost < costs[c_idx]
                if !(less_dist || less_cost) continue end
                delete!(unreached, c_idx) # Removed any reached goals
                if less_cost # Store new cost and achiever
                    costs[c_idx] = next_cost
                    achievers[c_idx] = is_axiom ? achievers[cond_idx] : act_idx
                end
                if !(c_idx in keys(queue)) # Enqueue new conditions
                    enqueue!(queue, c_idx, next_dist)
                elseif less_dist # Adjust distances
                    queue[c_idx] = next_dist
                    dists[c_idx] = next_dist
                end
            end
        end
    end

    # Return fact costs and achievers
    return costs, achievers
end

"Returns planning graph indices for initial facts."
function pgraph_init_idxs(graph::PlanningGraph, domain::Domain, state::State)
    @unpack conditions, cond_derived, cond_functional = graph
    # Handle non-derived initial conditions
    function check_cond(c, is_derived, is_func)
        is_derived && return false
        is_func && return satisfy(domain, state, c)::Bool
        c.name == :not && return !state[c.args[1]]::Bool
        return state[c]::Bool
    end
    init_idxs = broadcast(check_cond, conditions, cond_derived, cond_functional)
    # Handle derived initial conditions
    if length(PDDL.get_axioms(domain)) > 0
        init_idxs = pgraph_derived_idxs!(init_idxs, graph, domain, state)
    end
    return init_idxs
end

function pgraph_init_idxs(graph::PlanningGraph,
                          domain::Domain, state::GenericState)
    @unpack conditions, cond_derived, cond_functional = graph
    # Handle non-derived initial conditions
    function check_cond(c, is_derived, is_func)
        is_derived && return false
        is_func && return satisfy(domain, state, c)::Bool
        c.name == :not && return !(c.args[1] in PDDL.get_facts(state))
        return c in PDDL.get_facts(state)
    end
    init_idxs = broadcast(check_cond, conditions, cond_derived, cond_functional)
    # Handle derived initial conditions
    if !isempty(PDDL.get_axioms(domain))
        init_idxs = pgraph_derived_idxs!(init_idxs, graph, domain, state)
    end
    return init_idxs
end

"Determine indices for initial facts derived from axioms."
function pgraph_derived_idxs!(init_idxs::BitVector, graph::PlanningGraph,
                              domain::Domain, state::State)
    # Set up priority queue and condition flags
    queue = PriorityQueue{Int,Float32}(i => 0 for i in findall(init_idxs))
    condflags = [(UInt(1) << length(a.preconds)) - 1
                 for a in graph.actions[1:graph.n_axioms]]
    # Compute all initially true axioms
    while !isempty(queue)
        # Dequeue nearest fact/condition
        cond_idx = dequeue!(queue)
        # Iterate over child axioms
        for (act_idx, precond_idx) in graph.cond_children[cond_idx]
            is_axiom = act_idx <= graph.n_axioms
            if !is_axiom continue end
            # Skip actions with no children
            isempty(graph.act_children[act_idx]) && continue
            # Skip already achieved actions
            condflags[act_idx] === UInt(0) && continue
            # Set precondition flag for unachieved actions
            condflags[act_idx] &= ~(UInt(1) << (precond_idx-1))
            condflags[act_idx] !== UInt(0) && continue
            # Place child conditions on queue
            act_children = graph.act_children[act_idx]
            for c_idx in act_children
                init_idxs[c_idx] = true
                if !(c_idx in keys(queue)) # Enqueue new conditions
                    enqueue!(queue, c_idx, 0)
                end
            end
        end
    end
    # Return updated initial indices
    return init_idxs
end
