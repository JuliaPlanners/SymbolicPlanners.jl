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

"Precomputes a heuristic if necessary."
ensure_precomputed!(h::Heuristic, domain::Domain, state::State, spec::Specification) =
    !is_precomputed(h, domain, state, spec) ? precompute!(h, domain, state, spec) : h

"Computes the heuristic value of state relative to a goal in a given domain."
compute(h::Heuristic, domain::Domain, state::State, spec::Specification) =
    error("Not implemented.")
compute(h::Heuristic, domain::Domain, state::State, spec) =
    compute(h, domain, state, Specification(spec))

"Computes the heuristic value of state relative to a goal in a given domain."
function (h::Heuristic)(domain::Domain, state::State, spec::Specification;
                        use_cache::Bool=_use_heuristic_cache[])
    # Look-up cache if flag is enabled
    if use_cache
        key = (hash(h), domain.name, hash(state), hash(spec))
        val = get(_heuristic_cache, key, nothing)
        if (val !== nothing) return val end
    end
    # Precompute heuristic if necessary
    ensure_precomputed!(h, domain, state, spec)
    # Compute heuristic
    val = compute(h, domain, state, spec)
    # Store value in cache if flag is enabled
    if use_cache
        _heuristic_cache[key] = val
    end
    return val
end

(h::Heuristic)(domain::Domain, state::State, spec;
               use_cache::Bool=_use_heuristic_cache[]) =
    h(domain, state, Specification(spec); use_cache=use_cache)

include("precomputed.jl")
include("basic.jl")
include("pgraph.jl")
include("hsp.jl")
include("ff.jl")
include("reachability.jl")
