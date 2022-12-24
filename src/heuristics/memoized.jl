export MemoizedHeuristic, memoized

"Wraps an existing heursitic and memoizes heuristic computation."
struct MemoizedHeuristic{H <: Heuristic} <: Heuristic
    heuristic::H
    cache::Dict{NTuple{3,UInt}, Float32}
end

function MemoizedHeuristic(h::Heuristic)
    cache = Dict{NTuple{3,UInt}, Float32}()
    return MemoizedHeuristic(h, cache)
end

MemoizedHeuristic(h::MemoizedHeuristic) = h

Base.empty!(h::MemoizedHeuristic) = empty!(h.cache)

is_precomputed(h::MemoizedHeuristic) =
    is_precomputed(h.heuristic)

# TODO: CACHE PRECOMPUTED INFORMATION AS WELL
function precompute!(h::MemoizedHeuristic,
                     domain::Domain, state::State, spec::Specification)
    precompute!(h.heuristic, domain, state, spec)
    return h
end

function compute(h::MemoizedHeuristic,
                 domain::Domain, state::State, spec::Specification)
    key = (hash(domain), hash(state), hash(spec))
    val = get!(h.cache, key) do
        compute(h.heuristic, domain, state, spec)
    end
    return val
end

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

"Memoize heuristic values in a cache."
memoized(h::Heuristic) = MemoizedHeuristic(h)
