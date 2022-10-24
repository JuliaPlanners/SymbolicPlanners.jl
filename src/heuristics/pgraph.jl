# Planning graph utilities for relaxed heuristic computation

"Planning graph used by relaxation-based heuristics."
struct PlanningGraph
    n_axioms::Int # Number of ground actions converted from axioms
    n_goals::Int # Number of ground actions converted from goals
    actions::Vector{GroundAction} # All ground actions
    act_parents::Vector{Vector{Vector{Int}}} # Parent conditions of each action
    act_children::Vector{Vector{Int}} # Child conditions of each action
    conditions::Vector{Term} # All ground preconditions / goal conditions
    cond_children::Vector{Vector{Tuple{Int,Int}}} # Child actions of each condition
    cond_derived::BitVector # Whether the conditions are derived
    cond_functional::BitVector # Whether the conditions involve functions
end

"""
    build_planning_graph(domain, state, [goal::Term])

Construct planning graph for a `domain` grounded in a `state`, with an
optional `goal` formula that will be converted to action nodes.
"""
function build_planning_graph(domain::Domain, state::State,
                              goal::Union{Term,Nothing}=nothing)
    # Infer static and relevant fluents
    statfluents = infer_static_fluents(domain)
    relfluents = infer_relevant_fluents(domain, goal)
    # Populate list of ground actions and converted axioms
    actions = GroundAction[]
    # Add axioms converted to ground actions
    for (name, axiom) in pairs(PDDL.get_axioms(domain))
        if !(name in relfluents) continue end # Skip irrelevant axioms
        for ax in groundaxioms(domain, state, axiom; statics=statfluents)
            if ax.effect isa PDDL.GenericDiff
                push!(actions, ax)
            else # Handle conditional effects
                append!(actions, PDDL.flatten_conditions(ax))
            end
        end
    end
    n_axioms = length(actions)
    # Add ground actions, flattening conditional actions
    for act in groundactions(domain, state; statics=statfluents)
        # TODO: Eliminate redundant actions
        if act.effect isa PDDL.GenericDiff
            push!(actions, act)
        else # Handle conditional effects
            append!(actions, PDDL.flatten_conditions(act))
        end
    end
    # Add goals converted to ground actions
    n_goals = 0
    if !isnothing(goal)
        goal_actions = pgraph_goal_to_actions(domain, state, goal;
                                              statics=statfluents)
        n_goals = length(goal_actions)
        append!(actions, goal_actions)
    end
    # Ensure number of conditions doesn't exceed max limit of Sys.WORD_SIZE
    for act in actions
        if length(act.preconds) > Sys.WORD_SIZE
            resize!(act.preconds, Sys.WORD_SIZE)
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
    # Flatten map from conditions to child indices
    cond_children = collect(values(cond_map))
    conditions = collect(keys(cond_map))
    # Determine parent and child conditions of each action
    act_parents = [[Int[] for c in act.preconds] for act in actions]
    act_children = [Int[] for _ in 1:length(actions)]
    for (i, cond) in enumerate(conditions)
        # Collect parent conditions
        idxs = get(Vector{Tuple{Int,Int}}, cond_map, cond)
        for (act_idx, precond_idx) in idxs
            push!(act_parents[act_idx][precond_idx], i)
        end
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
    return PlanningGraph(n_axioms, n_goals, actions, act_parents, act_children,
                         conditions, cond_children, cond_derived, cond_functional)
end

"Converts a goal formula into ground actions in a planning graph."
function pgraph_goal_to_actions(domain::Domain, state::State, goal::Term;
                                statics=infer_static_fluents(domain))
    # Dequantify and simplify goal condition
    goal = PDDL.to_nnf(PDDL.dequantify(goal, domain, state, statics))
    goal = PDDL.simplify_statics(goal, domain, state, statics)
    # Convert goal condition to ground actions
    actions = GroundAction[]
    name = :goal
    term = Compound(:goal, Term[])
    if PDDL.is_dnf(goal) # Split disjunctive goal into multiple goal actions
        for clause in goal.args
            conds = PDDL.flatten_conjs(clause)
            act = GroundAction(name, term, conds, PDDL.GenericDiff())
            push!(actions, act)
        end
    else # Otherwise convert goal conditions to CNF form
        conds = PDDL.is_cnf(goal) ? goal.args : PDDL.to_cnf_clauses(goal)
        act = GroundAction(name, term, conds, PDDL.GenericDiff())
        push!(actions, act)
    end
    return actions
end

"Compute relaxed costs and paths to each fact node of a planning graph."
function relaxed_pgraph_search(domain::Domain, state::State, spec::Specification,
                               accum_op::Function, graph::PlanningGraph)
    # Initialize fact costs, precondition flags,  etc.
    n_actions = length(graph.actions)
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

    # Perform Djikstra / uniform-cost search until goals are reached
    goal_idx, goal_cost = nothing, Inf32
    first_goal_idx = n_actions - graph.n_goals
    while !isempty(queue) && isnothing(goal_idx)
        # Dequeue nearest fact/condition
        cond_idx = dequeue!(queue)
        # Iterate over child actions
        for (act_idx, precond_idx) in graph.cond_children[cond_idx]
            # Check if goal action is reached
            is_goal = act_idx > first_goal_idx
            # Skip actions with no children
            !is_goal && isempty(graph.act_children[act_idx]) && continue
            # Skip actions already achieved
            condflags[act_idx] === UInt(0) && continue
            # Set precondition flag for unachieved actions
            condflags[act_idx] &= ~(UInt(1) << (precond_idx-1))
            # Continue if preconditions for action are not fully satisfied
            condflags[act_idx] != UInt(0) && continue
            # Compute path cost and distance of achieved action or axiom
            act_parents = graph.act_parents[act_idx]
            is_axiom = act_idx <= graph.n_axioms
            if is_axiom # Axioms cost is the max of parent consts
                next_cost = maximum(act_parents) do precond_parents
                    minimum(costs[p] for p in precond_parents)
                end
                next_dist = dists[cond_idx]
            else # Lookup action cost if specified, default to one otherwise
                path_cost = accum_op(act_parents) do precond_parents
                    minimum(costs[p] for p in precond_parents)
                end
                act_cost = has_action_cost(spec) ?
                    get_action_cost(spec, graph.actions[act_idx].term) : 1
                next_cost = path_cost + act_cost
                next_dist = accum_op === maximum && !has_action_cost(spec) ?
                    next_cost : dists[cond_idx] + 1
            end
            if is_goal # Return goal index and goal cost
                goal_idx, goal_cost = act_idx, path_cost
                break
            end
            # Place child conditions on queue
            act_children = graph.act_children[act_idx]
            for c_idx in act_children
                less_dist = next_dist < dists[c_idx]
                less_cost = next_cost < costs[c_idx]
                if !(less_dist || less_cost) continue end
                if less_cost # Store new cost and achiever
                    costs[c_idx] = next_cost
                    achievers[c_idx] = act_idx
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

    # Return fact costs, achievers, goal index and cost
    return costs, achievers, goal_idx, goal_cost
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
