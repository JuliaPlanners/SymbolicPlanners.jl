export PrecomputedHeuristic, precomputed

"Heuristic that is already precomputed, and will not be precomputed again."
struct PrecomputedHeuristic{H <: Heuristic} <: Heuristic
    heuristic::H
end

function PrecomputedHeuristic(h::Heuristic, args...)
    h = ensure_precomputed!(h, args...)
    return PrecomputedHeuristic(h)
end

function PrecomputedHeuristic(h::PrecomputedHeuristic, args...)
    h = ensure_precomputed!(h.heuristic, args...)
    return PrecomputedHeuristic(h)
end

is_precomputed(h::PrecomputedHeuristic) = true

precompute!(h::PrecomputedHeuristic, ::Domain, ::State, ::Specification) = h

compute(h::PrecomputedHeuristic, domain::Domain, state::State, spec::Specification) =
    compute(h.heuristic, domain, state, spec)

"Precomputes a heuristic in advance, preventing recomputation later."
precomputed(h::Heuristic, domain::Domain, args...) =
    PrecomputedHeuristic(h, domain, args...)
