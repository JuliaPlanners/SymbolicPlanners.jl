## HSP family of heuristics ##
export HSPHeuristic, HAdd, HMax
export HSPRHeuristic, HAddR, HMaxR

"Precomputed domain information for HSP heuristic."
struct HSPCache
    domain::Domain # Preprocessed domain
    axioms::Vector{Clause} # Preprocessed axioms
    preconds::Dict{Symbol,Vector{Vector{Term}}} # Preconditions in DNF
    additions::Dict{Symbol,Vector{Term}} # Action add lists
end

"HSP family of delete-relaxation heuristics."
mutable struct HSPHeuristic <: Heuristic
    op::Function # Aggregator (e.g. maximum, sum) for fact costs
    cache::HSPCache # Precomputed domain information
    HSPHeuristic(op) = new(op)
    HSPHeuristic(op, cache) = new(op, cache)
end

Base.hash(heuristic::HSPHeuristic, h::UInt) =
    hash(heuristic.op, hash(HSPHeuristic, h))

function precompute!(heuristic::HSPHeuristic,
                     domain::Domain, state::State, spec::Specification)
    # Check if cache has already been computed
    if isdefined(heuristic, :cache) return heuristic end
    domain = copy(domain) # Make a local copy of the domain
    # Preprocess axioms
    axioms = regularize_clauses(domain.axioms) # Regularize domain axioms
    axioms = [Clause(ax.head, [t for t in ax.body if t.name != :not])
              for ax in axioms] # Remove negative literals
    domain.axioms = Clause[] # Remove axioms so they do not affect execution
    # Preprocess actions
    preconds = Dict{Symbol,Vector{Vector{Term}}}()
    additions = Dict{Symbol,Vector{Term}}()
    for (act_name, act_def) in domain.actions
        # Convert preconditions to DNF without negated literals
        conds = get_preconditions(act_def; converter=to_dnf)
        conds = [c.args for c in conds.args]
        for c in conds filter!(t -> t.name != :not, c) end
        preconds[act_name] = conds
        # Extract additions from each effect
        diff = effect_diff(act_def.effect)
        additions[act_name] = diff.add
    end
    heuristic.cache = HSPCache(domain, axioms, preconds, additions)
    return heuristic
end

function compute(heuristic::HSPHeuristic,
                 domain::Domain, state::State, spec::Specification)
    # Precompute if necessary
    if !isdefined(heuristic, :cache)
        precompute!(heuristic, domain, state, spec) end
    @unpack op, cache = heuristic
    @unpack domain = cache
    @unpack types, facts = state
    goals = get_goal_terms(spec)
    # Initialize fact costs in a GraphPlan-style graph
    fact_costs = Dict{Term,Float64}(f => 0 for f in facts)
    while true
        facts = Set(keys(fact_costs))
        state = State(types, facts, Dict{Symbol,Any}())
        if is_goal(spec, domain, state)
            return op([0; [fact_costs[g] for g in goals]]) end
        # Compute costs of one-step derivations of domain axioms
        for ax in cache.axioms
            _, subst = resolve(ax.body, [Clause(f, []) for f in facts])
            for s in subst
                body = [substitute(t, s) for t in ax.body]
                cost = op([0; [get(fact_costs, f, 0) for f in body]])
                derived = substitute(ax.head, s)
                if cost < get(fact_costs, derived, Inf)
                    fact_costs[derived] = cost end
            end
        end
        # Compute costs of all effects of available actions
        actions = available(state, domain)
        for act in actions
            act_args = domain.actions[act.name].args
            subst = Subst(var => val for (var, val) in zip(act_args, act.args))
            # Look-up preconds and substitute vars
            preconds = cache.preconds[act.name]
            preconds = [[substitute(t, subst) for t in c] for c in preconds]
            # Compute cost of reaching each action
            cost = minimum([[op([0; [get(fact_costs, f, 0) for f in conj]])
                             for conj in preconds]; Inf])
            # Compute cost of reaching each added fact
            additions = [substitute(a, subst) for a in cache.additions[act.name]]
            cost = cost + 1 # TODO: Handle arbitrary action costs
            for fact in additions
                if cost < get(fact_costs, fact, Inf)
                    fact_costs[fact] = cost end
            end
        end
        # Terminate if there's no change to the number of facts
        if length(fact_costs) == length(facts) && keys(fact_costs) == facts
            return Inf end
    end
end

"HSP heuristic where a fact's cost is the maximum cost of its dependencies."
HMax(args...) = HSPHeuristic(maximum, args...)

"HSP heuristic where a fact's cost is the summed cost of its dependencies."
HAdd(args...) = HSPHeuristic(sum, args...)

"HSPr family of delete-relaxation heuristics for regression search."
mutable struct HSPRHeuristic <: Heuristic
    op::Function
    fact_costs::Dict{Term,Float64} # Est. cost of reaching each fact from goal
    HSPRHeuristic(op) = new(op)
    HSPRHeuristic(op, fact_costs) = new(op, fact_costs)
end

Base.hash(heuristic::HSPRHeuristic, h::UInt) =
    hash(heuristic.op, hash(HSPRHeuristic, h))

function precompute!(heuristic::HSPRHeuristic,
                     domain::Domain, state::State, spec::Specification)
    @unpack op = heuristic
    @unpack types, facts = state
    goals = get_goal_terms(spec)
    # Preprocess domain and axioms
    domain = copy(domain)
    axioms = regularize_clauses(domain.axioms)
    axioms = [Clause(ax.head, [t for t in ax.body if t.name != :not])
              for ax in axioms]
    domain.axioms = Clause[]
    # Preprocess actions
    preconds = Dict{Symbol,Vector{Vector{Term}}}()
    additions = Dict{Symbol,Vector{Term}}()
    for (act_name, act_def) in domain.actions
        # Convert preconditions to DNF without negated literals
        conds = get_preconditions(act_def; converter=to_dnf)
        conds = [c.args for c in conds.args]
        for c in conds filter!(t -> t.name != :not, c) end
        preconds[act_name] = conds
        # Extract additions from each effect
        diff = effect_diff(act_def.effect)
        additions[act_name] = diff.add
    end
    # Initialize fact costs in a GraphPlan-style graph
    fact_costs = Dict{Term,Float64}(f => 0 for f in PDDL.get_facts(state))
    while true
        facts = Set(keys(fact_costs))
        state = State(types, facts, Dict{Symbol,Any}())
        # Compute costs of one-step derivations of domain axioms
        for ax in axioms
            _, subst = resolve(ax.body, [Clause(f, []) for f in facts])
            for s in subst
                body = [substitute(t, s) for t in ax.body]
                cost = op([0; [get(fact_costs, f, 0) for f in body]])
                derived = substitute(ax.head, s)
                if cost < get(fact_costs, derived, Inf)
                    fact_costs[derived] = cost end
            end
        end
        # Compute costs of all effects of available actions
        actions = available(state, domain)
        for act in actions
            act_args = domain.actions[act.name].args
            subst = Subst(var => val for (var, val) in zip(act_args, act.args))
            # Look-up preconds and substitute vars
            conds = preconds[act.name]
            conds = [[substitute(t, subst) for t in c] for c in conds]
            # Compute cost of reaching each action
            cost = minimum([[op([0; [get(fact_costs, f, 0) for f in conj]])
                             for conj in conds]; Inf])
            # Compute cost of reaching each added fact
            added = [substitute(a, subst) for a in additions[act.name]]
            cost = cost + 1 # TODO: Handle arbitrary action costs
            for fact in added
                if cost < get(fact_costs, fact, Inf)
                    fact_costs[fact] = cost end
            end
        end
        # Terminate when there's no change to the number of facts
        if length(fact_costs) == length(facts) && keys(fact_costs) == facts
            break end
    end
    heuristic.fact_costs = fact_costs
    return heuristic
end

function compute(heuristic::HSPRHeuristic,
                 domain::Domain, state::State, spec::Specification)
    # Precompute if necessary
    if !isdefined(heuristic, :fact_costs)
        precompute!(heuristic, domain, state, spec) end
    @unpack op, fact_costs = heuristic
    # Compute cost of achieving all facts in current state
    return op([0; [get(fact_costs, f, 0) for f in PDDL.get_facts(state)]])
end

"HSPr heuristic where a fact's cost is the maximum cost of its dependencies."
HMaxR(args...) = HSPRHeuristic(maximum, args...)

"HSPr heuristic where a fact's cost is the summed cost of its dependencies."
HAddR(args...) = HSPRHeuristic(sum, args...)
