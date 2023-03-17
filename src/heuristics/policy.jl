export PolicyValueHeuristic

"""
    PolicyValueHeuristic(policy::PolicySolution)

Wraps a `policy`, and returns the negated value estimate of a state 
(provided by [`get_value`](@ref)) as the heuristic goal-distance estimate.
"""
struct PolicyValueHeuristic{P <: PolicySolution} <: Heuristic
    policy::P
end

compute(h::PolicyValueHeuristic, ::Domain, state::State, ::Specification) =
    -get_value(h.policy, state)
