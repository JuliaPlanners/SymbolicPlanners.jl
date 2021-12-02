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
