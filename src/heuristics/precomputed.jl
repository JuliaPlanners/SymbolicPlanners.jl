export PrecomputedHeuristic, precomputed

"""
    PrecomputedHeuristic(heuristic::Heuristic, args...)

Wraps an existing `heuristic` and ensures that it is precomputed, preventing
repeated pre-computation on subsequent calls to [`precompute!`](@ref).
"""
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

function Base.show(io::IO, ::MIME"text/plain", h::PrecomputedHeuristic)
    indent = get(io, :indent, "")
    show_struct(io, h; indent = indent, show_fields = (:heuristic,))
end

is_precomputed(h::PrecomputedHeuristic) = true

precompute!(h::PrecomputedHeuristic, ::Domain, ::State, ::Specification) = h

compute(h::PrecomputedHeuristic, domain::Domain, state::State, spec::Specification) =
    compute(h.heuristic, domain, state, spec)

filter_available(h::PrecomputedHeuristic, domain::Domain, state::State, spec) =
    filter_available(h.heuristic, domain, state, spec)

filter_relevant(h::PrecomputedHeuristic, domain::Domain, state::State, spec) =
    filter_relevant(h.heuristic, domain, state, spec)

"""
    precomputed(h::Heuristic, domain::Domain, args...)

Precomputes a heuristic in advance, returning a [`PrecomputedHeuristic`](@ref)
that prevents repeated pre-computation later.
"""
precomputed(h::Heuristic, domain::Domain, args...) =
    PrecomputedHeuristic(h, domain, args...)
