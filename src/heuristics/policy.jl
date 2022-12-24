export PolicyValueHeuristic

"Computes distance to the goal based on policy value."
struct PolicyValueHeuristic{P <: PolicySolution} <: Heuristic
    policy::P
end

compute(h::PolicyValueHeuristic, ::Domain, state::State, ::Specification) =
    -get_value(h.policy, state)
