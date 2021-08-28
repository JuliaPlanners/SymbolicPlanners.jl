## Interface for planning heuristics ##
export Heuristic, precompute!, compute
export use_heuristic_cache!, clear_heuristic_cache!

"Global flag as to whether heuristic cache is enabled."
const _use_heuristic_cache = Ref(false)
"Globally enable or disable caching of heuristic values."
use_heuristic_cache!(val::Bool=true) = _use_heuristic_cache[] = val

"Cached heuristic values."
const _heuristic_cache = Dict{Tuple{UInt,Symbol,UInt,UInt}, Real}()
"Clear cache of heuristic values."
clear_heuristic_cache!() = empty!(_heuristic_cache)

"Abstract heuristic type, which defines the interface for planning heuristics."
abstract type Heuristic end

"Precomputes heuristic information given a domain, state, and goal."
precompute!(h::Heuristic, domain::Domain, state::State, spec::Specification) =
    h # Return the heuristic unmodified by default
precompute!(h::Heuristic, domain::Domain, state::State, spec) =
    precompute!(h, domain, state, Specification(spec))
precompute!(h::Heuristic, domain::Domain, state::State) =
    precompute!(h, domain, state, NullGoal())
precompute!(h::Heuristic, domain::Domain) =
    precompute!(h, domain, GenericState(Term[]), NullGoal())

"Returns whether heuristic has been precomputed for a domain, state, and goal."
is_precomputed(h::Heuristic, domain::Domain, state::State, spec::Specification) =
    false

"Computes the heuristic value of state relative to a goal in a given domain."
compute(h::Heuristic, domain::Domain, state::State, spec::Specification) =
    error("Not implemented.")
compute(h::Heuristic, domain::Domain, state::State, spec) =
    compute(h, domain, state, Specification(spec))

"Computes the heuristic value of state relative to a goal in a given domain."
function (h::Heuristic)(domain::Domain, state::State, spec::Specification;
                        cache::Bool=_use_heuristic_cache[])
    if (cache)
        key = (hash(h), domain.name, hash(state), hash(spec))
        if haskey(_heuristic_cache, key) return _heuristic_cache[key] end
    end
    val = compute(h, domain, state, spec)
    if (cache) _heuristic_cache[key] = val end
    return val
end

(h::Heuristic)(domain::Domain, state::State, spec;
               cache::Bool=_use_heuristic_cache[]) =
    h(domain, state, Specification(spec); cache=cache)

include("heuristics/basic.jl")
include("heuristics/hsp.jl")
include("heuristics/ff.jl")
