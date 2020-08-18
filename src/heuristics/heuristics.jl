## Interface for planning heuristics ##
export Heuristic, precompute, compute, clear_heuristic_cache!

"Cached heuristic values."
const heuristic_cache = Dict{Tuple{UInt,Symbol,UInt,UInt}, Real}()

"Clear cache of heuristic values."
clear_heuristic_cache!() = empty!(heuristic_cache)

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
                        cache::Bool=true)
    if (cache)
        key = (hash(h), domain.name, hash(state), hash(goal_spec))
        if haskey(heuristic_cache, key) return heuristic_cache[key] end
    end
    val = compute(h, domain, state, goal_spec)
    if (cache) heuristic_cache[key] = val end
    return val
end

(h::Heuristic)(domain::Domain, state::State, goal_spec; cache::Bool=true) =
    h(domain, state, GoalSpec(goal_spec); cache=cache)

include("basic.jl")
include("hsp.jl")
