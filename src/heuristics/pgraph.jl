# Planning graph utilities for relaxed heuristic computation

"Planning graph used by relaxation-based heuristics."
mutable struct PlanningGraph
    n_axioms::Int # Number of ground actions converted from axioms
    n_goals::Int # Number of ground actions converted from goals
    actions::Vector{GroundAction} # All ground actions
    act_parents::Vector{Vector{Vector{Int}}} # Parent conditions of each action
    act_children::Vector{Vector{Int}} # Child conditions of each action
    act_condflags::Vector{UInt} # Precondition bitflags for each action
    effect_map::Dict{Term,Vector{Int}} # Map of affected fluents to actions
    conditions::Vector{Term} # All ground preconditions / goal conditions
    cond_children::Vector{Vector{Tuple{Int,Int}}} # Child actions of each condition
    cond_derived::BitVector # Whether the conditions are derived
    cond_functional::BitVector # Whether the conditions involve functions
end

"""
    build_planning_graph(domain, state, [goal::Specification])

Construct planning graph for a `domain` grounded in a `state`, with an
optional `goal` specification that will be converted to action nodes.
"""
function build_planning_graph(
    domain::Domain, state::State, goal::Specification;
    kwargs...
)
    goal = Compound(:and, get_goal_terms(goal))
    return build_planning_graph(domain, state, goal; kwargs...)
end

function build_planning_graph(
    domain::Domain, state::State, goal::ActionGoal;
    kwargs...
)
    graph = build_planning_graph(domain, state; kwargs...)
    update_pgraph_goal!(graph, domain, state, goal)
    return graph
end

function build_planning_graph(
    domain::Domain, state::State, goal::Union{Term,Nothing}=nothing;
    statics = infer_static_fluents(domain),
    relevants = isnothing(goal) ? nothing : infer_relevant_fluents(domain, goal)
)
    # Populate list of ground actions and converted axioms
    actions = GroundAction[]
    # Add axioms converted to ground actions
    for (name, axiom) in pairs(PDDL.get_axioms(domain))
        if !isnothing(goal) && !(name in relevants)
            continue # Skip irrelevant axioms if goal is known
        end 
        for ax in groundaxioms(domain, state, axiom; statics=statics)
            if ax.effect isa PDDL.GenericDiff
                push!(actions, ax)
            else # Handle conditional effects
                append!(actions, PDDL.flatten_conditions(ax))
            end
        end
    end
    n_axioms = length(actions)
    # Add ground actions, flattening conditional actions
    for act in groundactions(domain, state; statics=statics)
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
                                              statics=statics)
        n_goals = length(goal_actions)
        append!(actions, goal_actions)
    end
    # Extract conditions and effects of ground actions
    cond_map = Dict{Term,Vector{Tuple{Int,Int}}}() # Map conditions to action indices
    effect_map = Dict{Term,Vector{Int}}() # Map effects to action indices
    for (i, act) in enumerate(actions)
        # Limit number of conditions to max limit of Sys.WORD_SIZE
        if length(act.preconds) > Sys.WORD_SIZE
            resize!(act.preconds, Sys.WORD_SIZE)
        end
        if isempty(act.preconds)
            push!(act.preconds, Const(true))
        end
        for (j, cond) in enumerate(act.preconds) # Preconditions
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
    act_parents = map(actions) do act 
        [Int[] for _ in 1:length(act.preconds)]
    end
    act_children = [Int[] for _ in actions]
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
    # Precompute initial bitflags for conditions
    act_condflags = map(a -> (UInt(1) << length(a.preconds) - 1), actions)
    # Determine if conditions are derived or functional
    cond_derived = isempty(PDDL.get_axioms(domain)) ?
        falses(length(conditions)) :
        broadcast(c -> PDDL.has_derived(c, domain), conditions)
    cond_functional = isempty(PDDL.get_functions(domain)) ?
        falses(length(conditions)) :
        broadcast(c -> PDDL.has_func(c, domain) ||
                       PDDL.has_global_func(c), conditions)
    # Construct and return graph
    return PlanningGraph(
        n_axioms, n_goals, actions, act_parents, act_children,
        act_condflags, effect_map, conditions, cond_children,
        cond_derived, cond_functional
    )
end

"Add a goal formula as one or more ground actions in a planning graph."
function pgraph_goal_to_actions(
    graph::PlanningGraph, domain::Domain, state::State, spec::Specification;
    statics=infer_static_fluents(domain)
)
    goal = Compound(:and, get_goal_terms(spec))
    return pgraph_goal_to_actions(domain, state, goal, statics=statics)
end

function pgraph_goal_to_actions(
    domain::Domain, state::State, goal::Term;
    statics=infer_static_fluents(domain)
)
    # Dequantify and simplify goal condition
    goal = PDDL.to_nnf(PDDL.dequantify(goal, domain, state, statics))
    goal = PDDL.simplify_statics(goal, domain, state, statics)
    # Convert goal condition to ground actions
    goal_actions = GroundAction[]
    name = :goal
    term = Compound(:goal, Term[])
    if PDDL.is_dnf(goal) # Split disjunctive goal into multiple goal actions
        for clause in goal.args
            conds = PDDL.flatten_conjs(clause)
            act = GroundAction(name, term, conds, PDDL.GenericDiff())
            push!(goal_actions, act)
        end
    else # Otherwise convert goal conditions to CNF form
        conds = PDDL.is_cnf(goal) ? goal.args : PDDL.to_cnf_clauses(goal)
        act = GroundAction(name, term, conds, PDDL.GenericDiff())
        push!(goal_actions, act)
    end
    return goal_actions
end

function pgraph_goal_to_actions(
    graph::PlanningGraph, domain::Domain, state::State, spec::ActionGoal;
    statics=nothing
)
    # Convert contstraints to CNF form
    constraints = PDDL.is_cnf(spec.constraints) ?
        spec.constraints : PDDL.to_cnf_clauses(spec.constraints)
    # Create goal action for each action that matches the specification
    goal_actions = GroundAction[]
    for orig_act in graph.actions
        # Check if action unifies with goal action
        unifiers = PDDL.unify(spec.action, orig_act.term)
        isnothing(unifiers) && continue
        conds = copy(orig_act.preconds)
        # Add constraints to preconditions
        for c in constraints
            c = isempty(unifiers) ? c : PDDL.substitute(c, unifiers)
            PDDL.is_ground(c) || continue # Skip non-ground constraints
            push!(conds, c)
        end
        act = GroundAction(:goal, orig_act.term, conds, PDDL.GenericDiff())
        push!(goal_actions, act)
    end
    return goal_actions
end

"Add or replace goal actions in a planning graph."
function update_pgraph_goal!(
    graph::PlanningGraph, domain::Domain, state::State, spec::Specification;
    statics=infer_static_fluents(domain)
)
    # Convert goal specification to ground actions
    goal_actions =
        pgraph_goal_to_actions(graph, domain, state, spec, statics=statics)
    # Replace old goal actions with new ones
    n_nongoals = length(graph.actions) - graph.n_goals
    resize!(graph.actions, n_nongoals)
    resize!(graph.act_parents, n_nongoals)
    resize!(graph.act_children, n_nongoals)
    resize!(graph.act_condflags, n_nongoals)
    graph.n_goals = length(goal_actions)
    goal_parents = map(goal_actions) do act 
        [Int[] for _ in 1:min(Sys.WORD_SIZE, length(act.preconds))]
    end
    goal_children = [Int[] for _ in goal_actions]
    append!(graph.actions, goal_actions)
    append!(graph.act_parents, goal_parents)
    append!(graph.act_children, goal_children)
    # Extract conditions of goal actions
    cond_map = Dict{Term,Vector{Tuple{Int,Int}}}()
    for (i, act) in enumerate(goal_actions)
        i += n_nongoals # Increment index by number of non-goal actions
        # Limit number of conditions to max limit of Sys.WORD_SIZE
        if length(act.preconds) > Sys.WORD_SIZE
            resize!(act.preconds, Sys.WORD_SIZE)
        end
        preconds = isempty(act.preconds) ? Term[Const(true)] : act.preconds
        for (j, cond) in enumerate(preconds)
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
    end
    # Precompute condition flags for goal actions
    append!(graph.act_condflags,
            map(a -> (UInt(1) << length(a.preconds) - 1), goal_actions))
    # Update children of existing conditions and parents of goal actions
    for (i, cond) in enumerate(graph.conditions)
        # Filter out old goal children
        cond_children = graph.cond_children[i]
        filter!(cond_children) do (act_idx, _) 
            act_idx <= n_nongoals
        end
        # Add new goals as children to existing condition
        child_idxs = get(Vector{Tuple{Int,Int}}, cond_map, cond)
        if !isempty(child_idxs)
            append!(cond_children, child_idxs)
            delete!(cond_map, cond)
        end
        # Update parent indices for each child goal
        for (act_idx, precond_idx) in child_idxs
            push!(graph.act_parents[act_idx][precond_idx], i)
        end
    end
    # Add any newly introduced conditions
    effect_map = graph.effect_map
    for (cond, child_idxs) in cond_map
        push!(graph.conditions, cond)
        push!(graph.cond_children, child_idxs)
        new_idx = length(graph.conditions)
        # Update parent indices for each child goal
        for (act_idx, precond_idx) in child_idxs
            push!(graph.act_parents[act_idx][precond_idx], new_idx)
        end
        # Update child indices for existing (non-goal) actions
        if cond.name in (:not, true, false) || PDDL.is_pred(cond, domain)
            idxs = get(Vector{Int}, effect_map, cond) # Handle literals
        else # Handle functional terms
            terms = PDDL.constituents(cond, domain)
            idxs = reduce(union, (get(Vector{Int}, effect_map, t) for t in terms))
        end
        push!.(graph.act_children[idxs], new_idx)
        # Flag whether condition is derived or functional
        is_derived = PDDL.has_derived(cond, domain)
        is_func = PDDL.has_func(cond, domain) || PDDL.has_global_func(cond)
        push!(graph.cond_derived, is_derived)
        push!(graph.cond_functional, is_func)
    end
    return graph
end

"Search state for relaxed planning graph search."
struct PlanningGraphSearchState
    init_conds::BitVector
    cond_dists::Vector{Float32}
    cond_costs::Vector{Float32}
    cond_achievers::Vector{Int}
    act_condflags::Vector{UInt64}
    queue::FastPriorityQueue{Int,Float32,DataStructures.FasterForward}
end

function PlanningGraphSearchState(graph::PlanningGraph)
    n_conds = length(graph.conditions)
    init_conds = falses(n_conds)
    cond_dists = zeros(Float32, n_conds)
    cond_costs = zeros(Float32, n_conds)
    cond_achievers = zeros(Int, n_conds)
    act_condflags = zeros(UInt, length(graph.actions))
    queue = FastPriorityQueue{Int,Float32}()
    return PlanningGraphSearchState(init_conds, cond_dists, cond_costs,
                                    cond_achievers, act_condflags, queue)
end

"Initialize search state for relaxed planning graph search."
function init_pgraph_search!(
    search_state::PlanningGraphSearchState, graph::PlanningGraph, 
    domain::Domain, state::State; compute_init_conds::Bool = true
)
    # Initialize condition distances and costs
    n_conds = length(graph.conditions)
    if n_conds != length(search_state.cond_dists)
        resize!(search_state.cond_dists, n_conds)
        resize!(search_state.cond_costs, n_conds)
        resize!(search_state.cond_achievers, n_conds)
    end
    fill!(search_state.cond_dists, Inf32)
    fill!(search_state.cond_costs, Inf32)
    fill!(search_state.cond_achievers, -1)
    # Initialize action precondition bitflags
    n_actions = length(graph.actions)
    if n_actions != length(search_state.act_condflags)
        resize!(search_state.act_condflags, n_actions)
    end
    copyto!(search_state.act_condflags, graph.act_condflags)
    # Compute facts which are true in the initial state
    init_conds = search_state.init_conds
    if compute_init_conds
        compute_pgraph_init_conds!(init_conds, graph, domain, state)
    end
    search_state.cond_dists[init_conds] .= 0.0f0
    search_state.cond_costs[init_conds] .= 0.0f0
    # Initialize priority queue
    empty!(search_state.queue)
    append!(search_state.queue, (i => 0.0f0 for i in findall(init_conds)))
    return search_state
end

"Compute planning graph indices for true initial facts."
function compute_pgraph_init_conds!(
    init_conds::BitVector, graph::PlanningGraph, domain::Domain, state::State
)
    @unpack conditions, cond_derived, cond_functional = graph
    # Handle non-derived initial conditions
    function check_cond(@nospecialize(c::Term), is_derived::Bool, is_func::Bool)
        is_derived && return false
        is_func && return satisfy(domain, state, c)::Bool
        c.name == :not && return !PDDL.get_fluent(state, c.args[1])::Bool
        c.name isa Bool && return c.name::Bool
        c isa Const && return PDDL.get_fluent(state, c)::Bool
        c isa Compound && return PDDL.get_fluent(state, c)::Bool
        return PDDL.get_fluent(state, c)::Bool
    end
    # Compute initial conditions with in-place broadcast
    broadcast!(check_cond, init_conds,
               conditions, cond_derived, cond_functional)
    # Handle derived initial conditions
    if length(PDDL.get_axioms(domain)) > 0
        compute_pgraph_derived_conds!(init_conds, graph)
    end
    return init_conds::BitVector
end

function compute_pgraph_init_conds!(
    init_conds::BitVector, graph::PlanningGraph,
    domain::Domain, state::GenericState
)
    @unpack conditions, cond_derived, cond_functional = graph
    # Handle non-derived initial conditions
    function check_cond(@nospecialize(c::Term), is_derived::Bool, is_func::Bool)
        is_derived && return false
        is_func && return satisfy(domain, state, c)::Bool
        c.name == :not && return !(c.args[1] in PDDL.get_facts(state))
        c.name isa Bool && return c.name::Bool
        return c in PDDL.get_facts(state)
    end
    # Compute initial conditions with in-place broadcast
    broadcast!(check_cond, init_conds,
               conditions, cond_derived, cond_functional)
    # Handle derived initial conditions
    if !isempty(PDDL.get_axioms(domain))
        compute_pgraph_derived_conds!(init_conds, graph)
    end
    return init_conds::BitVector
end

"Compute planning graph indices for initial facts derived from axioms."
function compute_pgraph_derived_conds!(
    init_conds::BitVector, graph::PlanningGraph
)
    # Set up search queue and condition flags
    queue = findall(init_conds)
    condflags = graph.act_condflags[1:graph.n_axioms]
    # Compute all initially true axioms
    while !isempty(queue)
        # Dequeue first fact/condition
        cond_idx = popfirst!(queue)
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
                if init_conds[c_idx] == false
                    init_conds[c_idx] = true
                    push!(queue, c_idx)
                end
            end
        end
    end
    # Return updated initial indices
    return init_conds::BitVector
end

"Compute relaxed costs and paths to each fact node of a planning graph."
function run_pgraph_search!(
    search_state::PlanningGraphSearchState, graph::PlanningGraph, 
    spec::Specification, accum_op::Function = maximum;
    action_costs = nothing
)
    # Unpack search state
    dists = search_state.cond_dists
    costs = search_state.cond_costs
    achievers = search_state.cond_achievers
    condflags = search_state.act_condflags
    queue = search_state.queue

    # Perform Djikstra / uniform-cost search until goals are reached
    goal_idx, goal_cost = nothing, Inf32
    last_nongoal_idx = length(graph.actions) - graph.n_goals
    while !isempty(queue) && isnothing(goal_idx)
        # Dequeue nearest fact/condition
        cond_idx, cond_dist = dequeue_pair!(queue)
        # Skip if distance greater than stored value
        cond_dist > dists[cond_idx] && continue
        # Iterate over child actions
        for (act_idx, precond_idx) in graph.cond_children[cond_idx]
            # Check if goal action is reached
            is_goal = act_idx > last_nongoal_idx
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
            if is_axiom # Axiom cost is the max of parent costs
                next_cost = maximum(act_parents) do precond_parents
                    minimum(costs[p] for p in precond_parents)
                end
                next_dist = dists[cond_idx]
            elseif is_goal # Goal cost is the accumulation of parent costs
                next_cost = accum_op(act_parents) do precond_parents
                    minimum(costs[p] for p in precond_parents)
                end
            else # Look up action cost if specified, default to one otherwise
                path_cost = accum_op(act_parents) do precond_parents
                    minimum(costs[p] for p in precond_parents)
                end
                if isnothing(action_costs)
                    act_cost = has_action_cost(spec) ?
                        get_action_cost(spec, graph.actions[act_idx].term) : 1
                else
                    act_cost = action_costs[act_idx]
                end
                next_cost = path_cost + act_cost
                next_dist = dists[cond_idx] + 1
            end
            # Return goal index and goal cost if goal is reached
            if is_goal
                goal_idx, goal_cost = act_idx, next_cost
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
                if less_dist # Adjust distances and place on queue
                    enqueue!(queue, c_idx, next_dist)
                    dists[c_idx] = next_dist
                end
            end
        end
    end

    # Return search state, goal index and cost
    return search_state, goal_idx, goal_cost
end

"Compute relaxed costs and paths to each fact node of a planning graph."
function run_pgraph_search(
    graph::PlanningGraph,  domain::Domain, state::State, spec::Specification,
    accum_op::Function = maximum; action_costs = nothing
)
    search_state = PlanningGraphSearchState(graph)
    init_pgraph_search!(search_state, graph, domain, state)
    return run_pgraph_search!(search_state, graph, spec, accum_op;
                              action_costs = action_costs)
end

