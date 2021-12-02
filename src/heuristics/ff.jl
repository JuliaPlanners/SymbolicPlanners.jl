## FastForward (FF) delete-relaxation heuristic ##
export FFHeuristic

"Precomputed domain information for FF heuristic."
struct FFCache
    domain::Domain # Preprocessed domain
    axioms::Vector{Clause} # Preprocessed axioms
    preconds::Dict{Symbol,Vector{Vector{Term}}} # Preconditions in DNF
    additions::Dict{Symbol,Vector{Term}} # Action add lists
end

"FastForward (FF) delete-relaxation heuristic."
mutable struct FFHeuristic <: Heuristic
    cache::FFCache # Precomputed domain information
    pre_key::UInt64 # Precomputation hash
    FFHeuristic() = new()
end

Base.hash(heuristic::FFHeuristic, h::UInt) = hash(FFHeuristic, h)

function precompute!(h::FFHeuristic,
                     domain::Domain, state::State, spec::Specification)
    # Check if cache has already been computed
    if is_precomputed(h, domain, state, spec) return h end
    h.pre_key = objectid(domain) # Precomputed data is unique to each domain
    domain = domain isa CompiledDomain ? # Make a local copy of the domain
        copy(PDDL.get_source(domain)) : copy(domain)
    # Preprocess axioms
    axioms = regularize_clauses(collect(values(domain.axioms)))
    axioms = [Clause(ax.head, [t for t in ax.body if t.name != :not])
              for ax in axioms] # Remove negative literals
    empty!(domain.axioms) # Remove axioms so they do not affect execution
    # Preprocess actions
    preconds = Dict{Symbol,Vector{Vector{Term}}}()
    additions = Dict{Symbol,Vector{Term}}()
    for (act_name, act_def) in domain.actions
        # Convert preconditions to DNF without negated literals
        conds = to_dnf(PDDL.get_precond(act_def))
        conds = [c.args for c in conds.args]
        for c in conds filter!(t -> t.name != :not, c) end
        preconds[act_name] = conds
        # Extract additions from each effect
        diff = effect_diff(domain, state, act_def.effect)
        additions[act_name] = diff.add
    end
    h.cache = FFCache(domain, axioms, preconds, additions)
    return h
end

function is_precomputed(h::FFHeuristic,
                        domain::Domain, state::State, spec::Specification)
    return (isdefined(h, :cache) && objectid(domain) == h.pre_key)
end

function compute(h::FFHeuristic,
                 domain::Domain, state::State, spec::Specification)
    # Precompute if necessary
    if !is_precomputed(h, domain, state, spec)
        precompute!(h, domain, state, spec) end
    @unpack cache = h
    @unpack domain = cache
    @unpack types, facts = state
    goals = get_goal_terms(spec)
    # Initialize fact levels and achievers in a GraphPlan-style planning graph
    levels = Dict{Term,Int}(f => 1 for f in facts)
    achievers = Dict{Term,Vector{Term}}(f => Term[] for f in facts)
    cur_level = 1
    while true
        facts = Set(keys(levels))
        state = GenericState(types, facts, Dict{Symbol,Any}())
        # Break out of loop once all goals are achieved
        if is_goal(spec, domain, state) break end
        cur_level += 1
        # Add all one-step derivations of domain axioms
        for ax in cache.axioms
            _, subst = resolve(ax.body, [Clause(f, []) for f in facts])
            for s in subst
                derived = substitute(ax.head, s)
                if haskey(levels, derived) continue end
                levels[derived] = cur_level
                achievers[derived] = Term[]
            end
        end
        # Add effects of available actions
        actions = available(domain, state)
        for act in actions
            act_vars = domain.actions[act.name].args
            subst = Subst(var => val for (var, val) in zip(act_vars, act.args))
            additions = [substitute(a, subst) for a in cache.additions[act.name]]
            for fact in additions
                if get(levels, fact, Inf) < cur_level continue end
                levels[fact] = cur_level # Update fact level
                push!(get!(achievers, fact, Term[]), act) # Add achieving actions
            end
        end
        # Terminate if we hit a fixed point before satisfying all goals
        if length(levels) == length(facts) && keys(levels) == facts
            return Inf end
    end
    # Backout relaxed partial-order plan from the planning graph
    plan = Set{Term}[]
    while cur_level > 1
        pushfirst!(plan, Set{Term}())
        next_goals = Set{Term}()
        for goal in goals
            if levels[goal] < cur_level
                push!(next_goals, goal) # Handle at next level
                continue
            end
            actions = achievers[goal] # Get all achieving actions
            if length(actions) == 0 continue end # No achievers, continue
            # TODO: match actions with their achieving preconditions
            act = actions[1] # Select the first achiever
            push!(plan[1], act) # Add to plan
            preconds = cache.preconds[act.name][1] # Select the first DNF clause
            act_vars = domain.actions[act.name].args
            s = Subst(var => val for (var, val) in zip(act_vars, act.args))
            preconds = [substitute(p, s) for p in preconds]
            union!(next_goals, preconds) # Handle preconds at next level
        end
        goals = next_goals
        cur_level -= 1
    end
    # Return length of relaxed plan
    # TODO : Return helpful actions
    return sum(length.(plan))
end
