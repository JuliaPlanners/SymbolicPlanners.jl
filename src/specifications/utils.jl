# Utilities for working with specifications

"""
$(SIGNATURES)

Simplifies a goal `spec` with respect to a `domain` and initial `state`.
Universal or existential conditions are expanded into conjuctions or
disjunctions, and predicates with statically known truth values are pruned.
"""
function simplify_goal(spec::Specification, domain::Domain, state::State;
                       statics=PDDL.infer_static_fluents(domain))
    goals = get_goal_terms(spec)
    if length(goals) == 1 && goals[1].name == Symbol("do-action")
        # Avoid simplifying action goals
        return spec
    else
        goals = simplify_goal(goals, domain, state, statics=statics)
        return set_goal_terms(spec, goals)
    end
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

function simplify_goal(spec::MultiGoalReward, domain::Domain, state::State;
                       statics=PDDL.infer_static_fluents(domain))
    goals = map(spec.goals) do goal
        simplify_goal(goal, domain, state, statics=statics)
    end
    return MultiGoalReward(goals, spec.rewards, spec.discount)
end
