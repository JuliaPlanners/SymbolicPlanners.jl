# Utilities for working with specifications

"""
$(SIGNATURES)

Simplifies a goal `spec` with respect to a `domain` and initial `state`.
Universal or existential conditions are expanded into conjuctions or
disjunctions, and predicates with statically known truth values are pruned.
"""
function simplify_goal(spec::Specification, domain::Domain, state::State)
    # Dequantify and simplify goal condition
    statics = PDDL.infer_static_fluents(domain)
    goal = Compound(:and, get_goal_terms(spec))
    goal = PDDL.to_nnf(PDDL.dequantify(goal, domain, state, statics))
    goal = PDDL.simplify_statics(goal, domain, state, statics)
    if PDDL.is_dnf(goal) && length(goal.args) == 1
        goal = goal.args[1]
    end
    terms = PDDL.flatten_conjs(goal)
    return set_goal_terms(spec, terms)
end