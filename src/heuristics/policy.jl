export PolicyHeuristic

"Computes distance to the goal based on policy value."
struct PolicyHeuristic{P <: PolicySolution} <: Heuristic
    policy::P
end

Base.hash(heuristic::PolicyHeuristic, h::UInt) =
    hash(heuristic.policy, hash(PolicyHeuristic, h))

compute(h::PolicyHeuristic, ::Domain, state::State, ::Specification) =
    -get_value(h.policy, state)
