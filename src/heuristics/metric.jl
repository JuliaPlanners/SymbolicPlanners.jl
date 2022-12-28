export MetricHeuristic, ManhattanHeuristic, EuclideanHeuristic

"""
    MetricHeuristic(metric, fluents[, coeffs])

Heuristic that computes a `metric` distance between the current state and the 
goals for the specified numeric `fluents`, which are (optionally) multiplied
by scalar `coeffs` before metric computation.

This heuristic can only be used with goal formulae that contain a list of 
equality constraints for the provided `fluents`.

# Arguments

  - `metric`
    
    Function that returns a scalar value given a vector of differences between
    the fluent values for the current state and the goal.

  - `fluents`
    
    A list of `Term`s that refer to numeric fluents in the state.

  - `coeffs`
    
    A list of scalar coefficients which each fluent value will be multiplied 
    by before metric computation. Defaults to `1` for all fluents.

"""
mutable struct MetricHeuristic{M} <: Heuristic
    metric::M
    fluents::Vector{Term}
    coeffs::Vector{Float32}
    goalvals::Vector{Float32}
    MetricHeuristic(metric::M, fluents, coeffs) where {M} =
        new{M}(metric, Vector{Term}(fluents), coeffs)
end
``
MetricHeuristic(metric, fluents) =
    MetricHeuristic(metric, fluents, ones(Float32, length(fluents)))

is_precomputed(h::MetricHeuristic) = isdefined(h, :goalvals)

function precompute!(h::MetricHeuristic,
                     domain::Domain, state::State, spec::Specification)
    goals = flatten_conjs(get_goal_terms(spec))
    h.goalvals = map(h.fluents) do f
        idxs  = findall(goals) do g
            g.name == :(==) && g.args[1] == f && g.args[2].name isa Real        
        end
        if isempty(idxs)
            error("Specification lacks an equality constraint for fluent $f")
        elseif length(idxs) > 1
            error("More than one equality constraint for fluent $f")
        end
        return Float32(goals[idxs[1]].args[2].name)
    end
    return h
end

function compute(h::MetricHeuristic,
                 domain::Domain, state::State, spec::Specification)
    # Compute differences for each fluent
    diffs = [c * (v - Float32(evaluate(domain, state, f)))
             for (f, c, v) in zip(h.fluents, h.coeffs, h.goalvals)]
    # Compute metric distance from fluent differences
    dist = h.metric(diffs)
    return dist
end

norm1(v) = sum(abs.(v))
norm2(v) = sqrt(sum(abs2.(v)))

"""
    ManhattanHeuristic(fluents[, coeffs])

Computes Manhattan distance to the goal for the specified numeric fluents. An
instance of [`MetricHeuristic`](@ref) which uses the L1 norm.
"""
const ManhattanHeuristic = MetricHeuristic{typeof(norm1)}

ManhattanHeuristic(fluents, args...) =
    MetricHeuristic(norm1, fluents, args...)

"""
    EuclideanHeuristic(fluents[, coeffs])

Computes Euclidean distance to the goal for the specified numeric fluents. An
instance of [`MetricHeuristic`](@ref) which uses the L2 norm.
"""
const EuclideanHeuristic = MetricHeuristic{typeof(norm2)}

EuclideanHeuristic(fluents, args...) =
    MetricHeuristic(norm2, fluents, args...)
