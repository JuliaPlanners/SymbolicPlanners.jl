## Interface for planning heuristics ##
export Heuristic, precompute, compute
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
precompute(h::Heuristic, domain::Domain, state::State, goal_spec::GoalSpec) =
    h # Return the heuristic unmodified by default
precompute(h::Heuristic, domain::Domain, state::State, goal_spec) =
    precompute(h, domain, state, GoalSpec(goal_spec))
precompute(h::Heuristic, domain::Domain, state::State) =
    precompute(h, domain, state, GoalSpec(goals=Term[]))
precompute(h::Heuristic, domain::Domain) =
    precompute(h, domain, State(Term[]), GoalSpec(goals=Term[]))

"Computes the heuristic value of state relative to a goal in a given domain."
compute(h::Heuristic, domain::Domain, state::State, goal_spec::GoalSpec) =
    error("Not implemented.")
compute(h::Heuristic, domain::Domain, state::State, goal_spec) =
    compute(h, domain, state, GoalSpec(goal_spec))

"Computes the heuristic value of state relative to a goal in a given domain."
function (h::Heuristic)(domain::Domain, state::State, goal_spec::GoalSpec;
                        cache::Bool=_use_heuristic_cache[])
    if (cache)
        key = (hash(h), domain.name, hash(state), hash(goal_spec))
        if haskey(_heuristic_cache, key) return _heuristic_cache[key] end
    end
    val = compute(h, domain, state, goal_spec)
    if (cache) _heuristic_cache[key] = val end
    return val
end

(h::Heuristic)(domain::Domain, state::State, goal_spec;
               cache::Bool=_use_heuristic_cache[]) =
    h(domain, state, GoalSpec(goal_spec); cache=cache)

include("basic.jl")
include("hsp.jl")
