export PruningHeuristic

"""
    PruningHeuristic(heuristic::Heuristic, pruner)

Combines an existing `heuristic` with `pruner`, an action pruning method that 
defines the [`filter_available`](@ref) and [`filter_relevant`](@ref) functions.
For example, `pruner` may be another heuristic that filters actions.
"""
struct PruningHeuristic{H <: Heuristic, P} <: Heuristic
    heuristic::H
    pruner::P
end

PruningHeuristic(h::PruningHeuristic, pruner) =
    PruningHeuristic(h.heuristic, pruner)
PruningHeuristic(h::Heuristic, pruner::PruningHeuristic) =
    PruningHeuristic(h.heuristic, pruner.pruner)
PruningHeuristic(h::PruningHeuristic, pruner::PruningHeuristic) =
    PruningHeuristic(h.heuristic, pruner.pruner)

function Base.show(io::IO, ::MIME"text/plain", h::PruningHeuristic)
    indent = get(io, :indent, "")
    show_struct(io, h; indent = indent, show_fields = (:heuristic, :pruner))
end
    
is_precomputed(h::PruningHeuristic) =
    is_precomputed(h.heuristic)
is_precomputed(h::PruningHeuristic{<:Heuristic, <:Heuristic}) =
    is_precomputed(h.heuristic) && is_precomputed(h.pruner)

function precompute!(h::PruningHeuristic,
                     domain::Domain, state::State, spec::Specification)
    precompute!(h.heuristic, domain, state, spec)
    return h
end

function precompute!(h::PruningHeuristic{<:Heuristic, <:Heuristic},
                     domain::Domain, state::State, spec::Specification)
    precompute!(h.heuristic, domain, state, spec)
    precompute!(h.pruner, domain, state, spec)
    return h
end

function precompute!(h::PruningHeuristic, domain::Domain, state::State)
    precompute!(h.heuristic, domain, state)
    return h
end

function precompute!(h::PruningHeuristic{<:Heuristic, <:Heuristic},
                     domain::Domain, state::State)
    precompute!(h.heuristic, domain, state)
    precompute!(h.pruner, domain, state)
    return h
end

function precompute!(h::PruningHeuristic, domain::Domain)
    precompute!(h.heuristic, domain)
    return h
end

function precompute!(h::PruningHeuristic{<:Heuristic, <:Heuristic},
                     domain::Domain)
    precompute!(h.heuristic, domain)
    precompute!(h.pruner, domain)
    return h
end

function compute(h::PruningHeuristic,
                 domain::Domain, state::State, spec::Specification)
    return compute(h.heuristic, domain, state, spec)
end

filter_available(h::PruningHeuristic, domain::Domain, state::State, spec) =
    filter_available(h.pruner, domain, state, spec)

filter_relevant(h::PruningHeuristic, domain::Domain, state::State, spec) =
    filter_relevant(h.pruner, domain, state, spec)
