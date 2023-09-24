export ActionGoal
export has_action_goal

"""
$(SIGNATURES)

Returns whether a specification has an action as a goal.
"""
has_action_goal(spec::Specification) = false

"""
    ActionGoal(action::Term, [constraints, step_cost])

[`Goal`](@ref) specification which requires that `action` is executed as the 
final step. Optionally, object `constraints` can be specified. These 
are either static constraints on the action's variable parameters, or predicates 
that must hold in the final state. The cost of each step in the solution
is `step_cost`, which defaults to 1.0.
"""
struct ActionGoal <: Goal
    action::Term # Action to be executed
    constraints::Vector{Term} # Constraints on action parameters
    step_cost::Float64 # Cost of each step
end

ActionGoal(action::Term) = ActionGoal(action, [], 1.0)
ActionGoal(action::Term, constraints) = ActionGoal(action, constraints, 1.0)
ActionGoal(action::Term, constraints::Term, step_cost) =
    ActionGoal(action, PDDL.flatten_conjs(constraints), step_cost)

Base.hash(spec::ActionGoal, h::UInt) =
    hash(spec.step_cost, hash(Set(spec.constraints), hash(spec.action, h)))
Base.:(==)(s1::ActionGoal, s2::ActionGoal) =
    s1.action == s2.action && Set(s1.constraints) == Set(s2.constraints) &&
    s1.step_cost == s2.step_cost

has_action_goal(spec::ActionGoal) = true

function is_goal(spec::ActionGoal, domain::Domain, state::State, action::Term)
    # Check if action that led to state unifies with goal action
    unifiers = PDDL.unify(spec.action, action)
    isnothing(unifiers) && return false
    # Check if constraints are satisfied
    isempty(spec.constraints) && return true
    constraints = isempty(unifiers) ?
        spec.constraints : [PDDL.substitute(c, unifiers) for c in spec.constraints]
    return satisfy(domain, state, constraints)
end

is_violated(spec::ActionGoal, domain::Domain, state::State) = false
get_cost(spec::ActionGoal, ::Domain, ::State, ::Term, ::State) = spec.step_cost
get_reward(spec::ActionGoal, ::Domain, ::State, ::Term, ::State) = -spec.step_cost

has_action_cost(spec::ActionGoal) = true
get_action_cost(spec::ActionGoal, ::Term) = spec.step_cost

function get_goal_terms(spec::ActionGoal)
    if isempty(spec.constraints)
        term = Compound(Symbol("do-action"), Term[spec.action])
    else
        term = Compound(Symbol("do-action"),
                        Term[spec.action, Compound(:and, spec.constraints)])
    end
    return Term[term]
end

function set_goal_terms(spec::ActionGoal, terms)
    @assert(length(terms) == 1 && terms[1].name == Symbol("do-action"),
            "Only `(do-action ?act ?constraints)` clauses allowed.")
    action = terms[1].args[1]
    constraints = length(terms[1].args) == 2 ? terms[1].args[2] : Term[]
    return ActionGoal(action, PDDL.flatten_conjs(constraints), spec.step_cost)
end
