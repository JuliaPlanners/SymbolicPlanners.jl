export ReachabilityHeuristic

"""
    ReachabilityHeuristic(max_steps::Int=100)

Heuristic which performs a reachability analysis for the goal via abstract
interpretation, returning an optimistic estimate of the number or cost of the
actions required to reach the goal, or `Inf` if the goal is not reached within
`max_steps` of abstract action execution.

For propositional domains (i.e. domains with no non-Boolean fluents), this
returns the same value as [`HMax`](@ref). For domains with numeric fluents or
other datatypes, this provides more informed estimates by performing abstract
interpretation of operations on those datatypes (e.g. interval arithmetic).
"""
mutable struct ReachabilityHeuristic <: Heuristic
    max_steps::Int
    absdom::Domain # Abstract domain
    ReachabilityHeuristic(max_steps) = new(max_steps)
end

ReachabilityHeuristic() = ReachabilityHeuristic(100)

is_precomputed(h::ReachabilityHeuristic) = isdefined(h, :absdom)

function precompute!(h::ReachabilityHeuristic,
                     domain::Domain, state::State, spec::Specification)
    # Store abstracted domain
    h.absdom, _ = abstracted(domain, state; autowiden=true)
    return h
end

function compute(h::ReachabilityHeuristic,
                 domain::Domain, state::State, spec::Specification)
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
    # Store abstracted domain, turn off automatic widening
    h.absdom, _ = abstracted(domain, state; autowiden=false)
    return h
end

function compute(h::ReachabilityHeuristic,
                 domain::Domain, state::State, spec::MinMetricGoal)
    # Get abstract domain and state
    absdom = h.absdom
    state = abstractstate(absdom, state)
    # Extract cost fluents (#TODO: use fixed has_subterm)
    cost_fluents = [f for f in PDDL.get_fluent_names(state)
                    if PDDL.has_name(spec.metric, (f.name,))]
    init_metric = absdom[state => spec.metric].lo
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
                cost_vals[i] = min(cost_vals[i], next_state[fluent].lo)
            end
            # Widen state variables
            accum_state = PDDL.widen!(absdom, accum_state, next_state)
        end
        # Narrow cost fluents
        for (i, fluent) in enumerate(cost_fluents)
            interval = PDDL.IntervalAbs{typeof(cost_vals[i])}(cost_vals[i])
            accum_state[fluent] = interval
        end
        # Terminate if fixpoint is reached
        if accum_state == state return Inf end
        state = accum_state
        # Compute new cost
        cur_metric = absdom[state => spec.metric].lo
        cost = cur_metric - init_metric
    end
    return cost
end
