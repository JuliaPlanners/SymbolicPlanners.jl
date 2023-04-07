export PolicyValueHeuristic, GoalDependentPolicyHeuristic

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

"""
    GoalDependentPolicyHeuristic(policies::Dict, [default])

Wraps a dictionary mapping planning [`Specification`](@ref)s to 
[`PolicySolution`](@ref)s. Given a particular specification, the heuristic
looks up the corresponding policy and returns its negated estimate of a state's
value as the heuristic goal-distance estimate.

If a `default` is provided, then this is used to construct a new policy
`policy = default(domain, state, spec)` for a specification `spec` that is
not found in the dictionary. Otherwise, an error is thrown.
"""
struct GoalDependentPolicyHeuristic{
    S <: Specification, P <: PolicySolution, F
} <: Heuristic
    policies::Dict{S, P}
    default::F
end

function GoalDependentPolicyHeuristic(
    policies::Dict{S, P}
) where {S <: Specification, P <: PolicySolution}
    return GoalDependentPolicyHeuristic{S, P, Nothing}(policies, nothing)
end

function compute(
    h::GoalDependentPolicyHeuristic{S, P, Nothing},
    domain::Domain, state::State, spec::Specification
) where {S, P}
    policy = get(h.policies, spec, nothing)
    policy === nothing && error("No policy for specification: $spec")
    return -get_value(policy, state)
end

function compute(
    h::GoalDependentPolicyHeuristic,
    domain::Domain, state::State, spec::Specification
)
    policy = get!(h.policies, spec) do 
        h.default(domain, state, spec)
    end
    return -get_value(policy, state)
end
