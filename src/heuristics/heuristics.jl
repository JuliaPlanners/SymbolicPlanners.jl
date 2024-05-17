## Interface for planning heuristics ##
export Heuristic, precompute!, compute
export is_precomputed, ensure_precomputed!

"""
    $(TYPEDEF)

Abstract type for search heuristics, which estimate the distance from a 
`State` to a goal specified by a [`Specification`](@ref). Once constructed, 
a `heuristic` can be called on a domain, state, and specification, returning a 
`Real` number (typically `Float32` for reduced memory usage).

    heuristic(domain, state, spec; precompute=true)

Heuristics may precompute and store information that will be used
repeatedly during search via the [`precompute!`](@ref) method. Evaluation of
the heuristic on a state is defined by [`compute`](@ref).

If the `precompute` keyword argument is true when calling `heuristic` as a
function, then [`precompute!`](@ref) will be called before [`compute`](@ref)
is called to perform the heuristic evaluation.
"""
abstract type Heuristic end

"""
$(SIGNATURES)

Precomputes heuristic information given a domain, state, and specification.
This function is typically called once during the initialization phase of
a [`Planner`](@ref)'s search algorithm.
"""
precompute!(h::Heuristic, domain::Domain, state::State, spec::Specification) =
    h # Return the heuristic unmodified by default
precompute!(h::Heuristic, domain::Domain, state::State, spec) =
    precompute!(h, domain, state, Specification(spec))
precompute!(h::Heuristic, domain::Domain, state::State) =
    precompute!(h, domain, state, NullGoal())
precompute!(h::Heuristic, domain::Domain) =
    precompute!(h, domain, GenericState(Term[]))

"""
$(SIGNATURES)

Returns whether heuristic has been precomputed.
"""
is_precomputed(h::Heuristic) = false

"""
$(SIGNATURES)

Precomputes a heuristic if necessary.
"""
ensure_precomputed!(h::Heuristic, args...) =
    !is_precomputed(h) ? precompute!(h, args...) : h

"""
$(SIGNATURES)

Computes the heuristic value of state relative to a goal in a given domain.
"""
compute(h::Heuristic, domain::Domain, state::State, spec::Specification) =
    error("Not implemented.")
compute(h::Heuristic, domain::Domain, state::State, spec) =
    compute(h, domain, state, Specification(spec))

"""
$(SIGNATURES)

Uses heuristic information to filter the set of available actions at a
given state. Defaults to returning `available(domain, state)`.
"""
filter_available(h::Heuristic, domain::Domain, state::State, spec) =
    available(domain, state)

"""
$(SIGNATURES)

Uses heuristic information to filter the set of relevant actions at a
given state. Defaults to returning `relevant(domain, state)`.
"""
filter_relevant(h::Heuristic, domain::Domain, state::State, spec) =
    relevant(domain, state)

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
include("pruning.jl")
include("basic.jl")
include("metric.jl")
include("planner.jl")
include("policy.jl")
include("pgraph.jl")
include("hsp.jl")
include("ff.jl")
include("reachability.jl")
include("lmcut.jl")
