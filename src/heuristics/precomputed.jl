export PrecomputedHeuristic, precomputed

"Heuristic that is already precomputed, and will not be precomputed again."
struct PrecomputedHeuristic{H <: Heuristic} <: Heuristic
    heuristic::H
end

function PrecomputedHeuristic(h::Heuristic, args...)
    h = precompute!(h, args...)
    return PrecomputedHeuristic(h)
end

Base.hash(heuristic::PrecomputedHeuristic, h::UInt) =
    hash(PrecomputedHeuristic, hash(heuristic.heuristic, h))

precompute!(h::PrecomputedHeuristic, ::Domain, ::State, ::Specification) =
    h

is_precomputed(h::PrecomputedHeuristic, ::Domain, ::State, ::Specification) =
    true

compute(h::PrecomputedHeuristic, domain::Domain, state::State, spec::Specification) =
    compute(h.heuristic, domain, state, spec)

"Precomputes a heuristic in advance, preventing recomputation later."
precomputed(h::Heuristic, domain::Domain, args...) =
    PrecomputedHeuristic(h, domain, args...)
