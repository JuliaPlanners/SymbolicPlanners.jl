export SimplifiedGoal, simplified, is_simplified, simplify_goal

"""
    SimplifiedGoal(spec::Specification)

Wraps an existing `spec` and ensures that its goal terms are simplified,
preventing repeated simplification on further calls to [`simplified`](@ref).
"""
struct SimplifiedGoal{S <: Specification} <: Specification
    spec::S # Underlying specification
end

function Base.show(io::IO, ::MIME"text/plain", spec::SimplifiedGoal)
    indent = get(io, :indent, "")
    show_struct(io, spec; indent = indent, show_fields=(:spec,))
end

Base.hash(spec::SimplifiedGoal, h::UInt) =
    hash(spec.spec, h)
Base.:(==)(s1::SimplifiedGoal, s2::SimplifiedGoal) =
    s1.spec == s2.spec

@set_subspec(SimplifiedGoal, spec)

"""
    simplified(spec::Specification, domain::Domain, state::State)

Simplifies a `spec` with respect to a `domain` and initial `state` via
[`simplify_goal`](@ref), returning a [`SimplifiedGoal`](@ref) that prevents
repeated simplification.
"""
function simplified(spec::Specification, domain::Domain, state::State; kwargs...)
    if is_simplified(spec)
        return spec
    else
        return SimplifiedGoal(simplify_goal(spec, domain, state; kwargs...))
    end
end

"""
$(SIGNATURES)

Returns whether a specification has been simplified.
"""
is_simplified(spec::Specification) =
    has_subspec(spec) ? is_simplified(get_subspec(spec)) : false
is_simplified(spec::SimplifiedGoal) =
    true
is_simplified(spec::ActionGoal) =
    true

"""
    simplify_goal(spec::Specification, domain::Domain, state::State; kwargs...)

Simplifies a goal `spec` with respect to a `domain` and initial `state`.
Universal or existential conditions are expanded into conjuctions or
disjunctions, and predicates with statically known truth values are pruned.
The `statics` keyword can be specified to provide a list of static fluents.
"""
function simplify_goal(spec::Specification, domain::Domain, state::State;
                       kwargs...)
    if has_subspec(spec)
        subspec = get_subspec(spec)
        new_subspec = simplify_goal(subspec, domain, state; kwargs...)
        return new_subspec === subspec ? spec : set_subspec(spec, new_subspec)
    elseif has_action_goal(spec)
        return spec
    else
        goals = get_goal_terms(spec)
        goals = simplify_goal(goals, domain, state; kwargs...)
        return set_goal_terms(spec, goals)
    end
end
simplify_goal(spec::SimplifiedGoal, domain::Domain, state::State; kwargs...) =
    spec

function simplify_goal(spec::MultiGoalReward, domain::Domain, state::State;
                       statics=PDDL.infer_static_fluents(domain))
    goals = map(spec.goals) do goal
        simplify_goal(goal, domain, state, statics=statics)
    end
    return MultiGoalReward(goals, spec.rewards, spec.discount)
end

function simplify_goal(goal::Term, domain::Domain, state::State;
                       statics=PDDL.infer_static_fluents(domain))
    goal = PDDL.to_nnf(PDDL.dequantify(goal, domain, state, statics))
    goal = PDDL.simplify_statics(goal, domain, state, statics)
    return goal
end

function simplify_goal(goals::AbstractVector{<:Term}, domain::Domain, state::State;
                       statics=PDDL.infer_static_fluents(domain))
    goal = Compound(:and, goals)
    goal = simplify_goal(goal, domain, state, statics=statics)
    return PDDL.flatten_conjs(goal)
end
