## Interface for planning heuristics ##
export Heuristic, precompute!, compute

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
    precompute!(h, domain, GenericState(Term[]))

"Returns whether heuristic has been precomputed."
is_precomputed(h::Heuristic) = false

"Precomputes a heuristic if necessary."
ensure_precomputed!(h::Heuristic, args...) =
    !is_precomputed(h) ? precompute!(h, args...) : h

"Computes the heuristic value of state relative to a goal in a given domain."
compute(h::Heuristic, domain::Domain, state::State, spec::Specification) =
    error("Not implemented.")
compute(h::Heuristic, domain::Domain, state::State, spec) =
    compute(h, domain, state, Specification(spec))

"Computes the heuristic value of state relative to a goal in a given domain."
function (h::Heuristic)(domain::Domain, state::State, spec::Specification;
                        precompute::Bool=true)
    # Precompute heuristic if necessary
    h = precompute ? precompute!(h, domain, state, spec) :
        ensure_precomputed!(h, domain, state, spec)
    # Compute heuristic
    return compute(h, domain, state, spec)
end

function (h::Heuristic)(domain::Domain, state::State, spec;
                        precompute::Bool=true)
    return h(domain, state, Specification(spec); precompute=precompute)
end

function (h::Heuristic)(domain::Domain, problem::Problem; precompute::Bool=true)
    state = initstate(domain, problem)
    spec = Specification(problem)
    return h(domain, state, spec; precompute=precompute)
end

include("memoized.jl")
include("precomputed.jl")
include("basic.jl")
include("metric.jl")
include("planner.jl")
include("policy.jl")
include("pgraph.jl")
include("hsp.jl")
include("ff.jl")
include("reachability.jl")
