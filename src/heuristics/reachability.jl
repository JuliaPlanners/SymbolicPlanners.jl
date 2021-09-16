export ReachabilityHeuristic

"Generalized reachability heuristic."
mutable struct ReachabilityHeuristic <: Heuristic
    max_steps::Int
    pre_key::Tuple{UInt,UInt} # Key to check if information needs to be precomputed again
    absdom::Domain # Abstract domain
    ReachabilityHeuristic(max_steps) = new(max_steps)
end

ReachabilityHeuristic() = ReachabilityHeuristic(100)

Base.hash(heuristic::ReachabilityHeuristic, h::UInt) =
    hash(heuristic.op, hash(ReachabilityHeuristic, h))

function precompute!(h::ReachabilityHeuristic,
                     domain::Domain, state::State, spec::Specification)
    # Check if cache has already been computed
    if is_precomputed(h, domain, state, spec) return h end
     # Precomputed data is unique to each domain and specification
    h.pre_key = (objectid(domain), hash(spec))
    # Store abstracted domain
    h.absdom, _ = abstracted(domain, state)
    return h
end

function is_precomputed(h::ReachabilityHeuristic,
                        domain::Domain, state::State, spec::Specification)
    return (isdefined(h, :pre_key) &&
            objectid(domain) == h.pre_key[1] && hash(spec) == h.pre_key[2])
end

function compute(h::ReachabilityHeuristic,
                 domain::Domain, state::State, spec::Specification)
    # Precompute if necessary
    if !is_precomputed(h, domain, state, spec)
        precompute!(h, domain, state, spec) end
    # Get abstract domain and state
    absdom = h.absdom
    state = abstractstate(absdom, state)
    # Iterate until we reach the goal or a fixpoint
    steps = 0
    while steps < h.max_steps
        # Terminate if goal is achieved or fixpoint is reached
        if is_goal(spec, absdom, state) return steps end
        steps += 1
        # Apply all available actions in abstract domain
        next_state = state
        for act in available(absdom, state)
            # Execute abstract action with automatic widening
            next_state = execute(absdom, next_state, act, check=false)
        end
        # Terminate if fixpoint is reached
        if next_state == state return Inf end
        state = next_state
    end
    return steps
end

# Specializations for metric minimization

function precompute!(h::ReachabilityHeuristic,
                     domain::Domain, state::State, spec::MinMetricGoal)
    # Check if cache has already been computed
    if is_precomputed(h, domain, state, spec) return h end
    # Precomputed data is unique to each domain and specification
    h.pre_key = (objectid(domain), hash(spec))
    # Store abstracted domain, turn off automatic widening
    h.absdom, _ = abstracted(domain, state; autowiden=false)
    return h
end

function compute(h::ReachabilityHeuristic,
                 domain::Domain, state::State, spec::MinMetricGoal)
    # Precompute if necessary
    if !is_precomputed(h, domain, state, spec)
        precompute!(h, domain, state, spec) end
    # Get abstract domain and state
    absdom = h.absdom
    state = abstractstate(absdom, state)
    # Extract cost fluents (#TODO: use fixed has_subterm)
    cost_fluents = [f for f in PDDL.get_fluent_names(state)
                    if PDDL.has_name(spec.metric, (f.name,))]
    init_metric = absdom[state => spec.metric].interval.lo
    cost = 0.0
    # Iterate until we reach the goal or a fixpoint
    steps = 0
    while steps <= h.max_steps
        # Terminate if goal is achieved or fixpoint is reached
        if is_goal(spec, absdom, state) return cost end
        steps += 1
        # Apply all available actions in abstract domain (with widening)
        accum_state = copy(state)
        cost_vals = [Inf for f in cost_fluents]
        for act in available(absdom, state)
            # Execute abstract action
            next_state = execute(absdom, state, act, check=false)
            # Compute cost fluent values
            for (i, fluent) in enumerate(cost_fluents)
                interval = next_state[fluent].interval
                cost_vals[i] = min(cost_vals[i], interval.lo)
            end
            # Widen state variables
            accum_state = widen!(absdom, accum_state, next_state)
        end
        # Narrow cost fluents
        for (i, fluent) in enumerate(cost_fluents)
            interval = PDDL.IntervalAbs(cost_vals[i])
            accum_state[fluent] = interval
        end
        # Terminate if fixpoint is reached
        if accum_state == state return Inf end
        state = accum_state
        # Compute new cost
        cur_metric = absdom[state => spec.metric].interval.lo
        cost = cur_metric - init_metric
    end
    return cost
end
