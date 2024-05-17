export MemoizedHeuristic, memoized

"""
    MemoizedHeuristic(heuristic::Heuristic)

Wraps an existing heuristic and memoizes heuristic evaluations in a hash table.
"""
struct MemoizedHeuristic{H <: Heuristic} <: Heuristic
    heuristic::H
    cache::Dict{NTuple{3,UInt}, Float32}
end

function MemoizedHeuristic(h::Heuristic)
    cache = Dict{NTuple{3,UInt}, Float32}()
    return MemoizedHeuristic(h, cache)
end

MemoizedHeuristic(h::MemoizedHeuristic) = h

function Base.show(io::IO, ::MIME"text/plain", h::MemoizedHeuristic)
    indent = get(io, :indent, "")
    show_struct(io, h; indent = indent, show_fields = (:heuristic,))
end

Base.empty!(h::MemoizedHeuristic) = empty!(h.cache)

is_precomputed(h::MemoizedHeuristic) =
    is_precomputed(h.heuristic)

# TODO: CACHE PRECOMPUTED INFORMATION AS WELL
function precompute!(h::MemoizedHeuristic,
                     domain::Domain, state::State, spec::Specification)
    precompute!(h.heuristic, domain, state, spec)
    return h
end

precompute!(h::MemoizedHeuristic, domain::Domain, state::State) =
    (precompute!(h.heuristic, domain, state); h)

precompute!(h::MemoizedHeuristic, domain::Domain) =
    (precompute!(h.heuristic, domain); h)

function compute(h::MemoizedHeuristic,
                 domain::Domain, state::State, spec::Specification)
    key = (hash(domain), hash(state), hash(spec))
    val = get!(h.cache, key) do
        compute(h.heuristic, domain, state, spec)
    end
    return val
end

filter_available(h::MemoizedHeuristic, domain::Domain, state::State, spec) =
    filter_available(h.heuristic, domain, state, spec)

filter_relevant(h::MemoizedHeuristic, domain::Domain, state::State, spec) =
    filter_relevant(h.heuristic, domain, state, spec)

function (h::MemoizedHeuristic)(domain::Domain, state::State, spec::Specification;
                                precompute::Bool=true)
    key = (hash(domain), hash(state), hash(spec))
    val = get!(h.cache, key) do
        h = precompute ? precompute!(h, domain, state, spec) :
            ensure_precomputed!(h, domain, state, spec)
        return compute(h.heuristic, domain, state, spec)
    end
    return val
end

"""
    memoized(h::Heuristic)

Constructs a memoized version of `h` which caches outputs in a hash table after
each evaluation of the heuristic on a new domain, state, or specification.
"""
memoized(h::Heuristic) = MemoizedHeuristic(h)
