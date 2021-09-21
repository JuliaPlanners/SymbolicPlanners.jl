## HSP family of heuristics ##
export HSPHeuristic, HAdd, HMax
export HSPRHeuristic, HAddR, HMaxR

"Precomputed domain information for HSP heuristic."
struct HSPCache
    domain::Domain # Preprocessed domain
    depgraph::DependencyGraph # Domain dependency graph
end

"HSP family of delete-relaxation heuristics."
mutable struct HSPHeuristic <: Heuristic
    op::Function # Aggregator (e.g. maximum, sum) for fact costs
    cache::HSPCache # Precomputed domain information
    pre_key::UInt64 # Key to check if information needs to be precomputed again
    HSPHeuristic(op) = new(op)
end

Base.hash(heuristic::HSPHeuristic, h::UInt) =
    hash(heuristic.op, hash(HSPHeuristic, h))

function precompute!(h::HSPHeuristic,
                     domain::Domain, state::State, spec::Specification)
    # Check if cache has already been computed
    if is_precomputed(h, domain, state, spec) return h end
    h.pre_key = objectid(domain) # Precomputed data is unique to each domain
    # Store abstracted domain and dependency graph
    absdom = abstracted(domain; abstractions=Dict())
    h.cache = HSPCache(absdom, dependency_graph(domain))
    return h
end

function precompute!(h::HSPHeuristic,
                     domain::CompiledDomain, state::State, spec::Specification)
    # Check if cache has already been computed
    if is_precomputed(h, domain, state, spec) return h end
    h.pre_key = objectid(domain) # Precomputed data is unique to each domain
    # Abstract source domain then recompile
    absdom, _ = abstracted(domain, state)
    depgraph = dependency_graph(PDDL.get_source(domain))
    h.cache = HSPCache(absdom, depgraph)
    return h
end

function is_precomputed(h::HSPHeuristic,
                        domain::Domain, state::State, spec::Specification)
    return (isdefined(h, :cache) && objectid(domain) == h.pre_key)
end

function compute(h::HSPHeuristic,
                 domain::Domain, state::State, spec::Specification)
    # Precompute if necessary
    if !is_precomputed(h, domain, state, spec)
        precompute!(h, domain, state, spec) end
    @unpack op, cache = h
    @unpack domain, depgraph = cache
    goals = get_goal_terms(spec)
    # Abstract state, initialize fact costs in a GraphPlan-style graph
    state = abstractstate(domain, state)
    fact_costs = Dict{Term,Float64}(f => 0 for f in PDDL.get_facts(state))
    # Iterate until we reach the goal or a fixpoint
    while true
        # Terminate if goal is roached
        if is_goal(spec, domain, state)
            return reduce(op, (fact_costs[g] for g in goals), init=0)
        end
        # Compute costs of one-step derivations of domain axioms
        # TODO: derivable(domain, state)
        # Compute costs of all effects of available actions
        next_state = copy(state)
        for act in available(domain, state)
            act_vars = PDDL.get_actions(domain)[act.name] |> PDDL.get_argvars
            subst = Subst(var => val for (var, val) in zip(act_vars, act.args))
            # Look-up parent conditions and substitute vars
            parents = depgraph.actions[act.name].parents
            parents = ((substitute(t, subst) for t in c) for c in parents)
            # Compute cost of reaching each action
            conj_costs = (reduce(op, (get(fact_costs, f, 0) for f in c), init=0)
                          for c in parents)
            cost = reduce(min, conj_costs, init=Inf)
            # Compute cost of reaching each child effect
            children = depgraph.actions[act.name].children
            cost = cost + 1 # TODO: Handle arbitrary action costs
            for f in (substitute(t, subst) for t in children)
                if cost < get(fact_costs, f, Inf) fact_costs[f] = cost end
            end
            # Execute action and widen state
            next_state = execute!(domain, next_state, act, check=false)
        end
        # Terminate if fixpoint is reached
        if next_state == state return Inf end
        state = next_state
    end
end

"HSP heuristic where a fact's cost is the maximum cost of its dependencies."
HMax(args...) = HSPHeuristic(max, args...)

"HSP heuristic where a fact's cost is the summed cost of its dependencies."
HAdd(args...) = HSPHeuristic(+, args...)

"HSPr family of delete-relaxation heuristics for regression search."
mutable struct HSPRHeuristic <: Heuristic
    op::Function
    fact_costs::Dict{Term,Float64} # Est. cost of reaching each fact from goal
    pre_key::NTuple{3,UInt64} # Key to check if precomputation needs to re-run
    HSPRHeuristic(op) = new(op)
end

Base.hash(heuristic::HSPRHeuristic, h::UInt) =
    hash(heuristic.op, hash(HSPRHeuristic, h))

function precompute!(h::HSPRHeuristic,
                     domain::Domain, state::State, spec::Specification)
    # Check if data has already been computed
    if is_precomputed(h, domain, state, spec) return h end
    # Precomputed data is tied to all three inputs
    h.pre_key = (objectid(domain), hash(state), hash(spec))
    # Initialize fact costs in a GraphPlan-style graph
    fact_costs = Dict{Term,Float64}(f => 0 for f in keys(state)
                                    if PDDL.is_pred(f, domain))
    # Abstract domain and compute dependency graph
    if domain isa CompiledDomain
        depgraph = dependency_graph(PDDL.get_source(domain))
    else
        depgraph = dependency_graph(domain)
    end
    domain, state = abstracted(domain, state)
    # Iterate until we reach the goal or a fixpoint
    while true
        # Compute costs of one-step derivations of domain axioms
        # TODO: derivable(domain, state)
        # Compute costs of all effects of available actions
        next_state = state
        for act in available(domain, state)
            act_vars = PDDL.get_actions(domain)[act.name] |> PDDL.get_argvars
            subst = Subst(var => val for (var, val) in zip(act_vars, act.args))
            # Look-up parent conditions and substitute vars
            parents = depgraph.actions[act.name].parents
            parents = ((substitute(t, subst) for t in c) for c in parents)
            # Compute cost of reaching each action
            conj_costs = (reduce(h.op, (get(fact_costs, f, 0) for f in c), init=0)
                          for c in parents)
            cost = reduce(min, conj_costs, init=Inf)
            # Compute cost of reaching each child effect
            children = depgraph.actions[act.name].children
            cost = cost + 1 # TODO: Handle arbitrary action costs
            for f in (substitute(t, subst) for t in children)
                if cost < get(fact_costs, f, Inf) fact_costs[f] = cost end
            end
            # Execute action and widen state
            next_state = execute(domain, next_state, act, check=false)
        end
        # Terminate if  fixpoint is reached
        if next_state == state break end
        state = next_state
    end
    h.fact_costs = fact_costs
    return h
end

function is_precomputed(h::HSPRHeuristic,
                        domain::Domain, state::State, spec::Specification)
    return (isdefined(h, :fact_costs) && objectid(domain) == h.pre_key[1]
            && hash(state) == h.pre_key[2] && hash(spec) == h.pre_key[3])
end

function compute(h::HSPRHeuristic,
                 domain::Domain, state::State, spec::Specification)
    # Compute cost of achieving all facts in current state
    facts = PDDL.get_facts(state)
    return reduce(h.op, (get(h.fact_costs, f, 0) for f in facts), init=0)
end

"HSPr heuristic where a fact's cost is the maximum cost of its dependencies."
HMaxR(args...) = HSPRHeuristic(max, args...)

"HSPr heuristic where a fact's cost is the summed cost of its dependencies."
HAddR(args...) = HSPRHeuristic(+, args...)
