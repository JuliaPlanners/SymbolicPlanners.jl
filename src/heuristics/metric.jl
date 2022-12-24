export MetricHeuristic, ManhattanHeuristic, EuclideanHeuristic

"Computes metric distance to the goal for the specified numeric fluents."
mutable struct MetricHeuristic{M} <: Heuristic
    metric::M
    fluents::Vector{Term}
    coeffs::Vector{Float32}
    goalvals::Vector{Float32}
    MetricHeuristic(metric::M, fluents, coeffs) where {M} =
        new{M}(metric, Vector{Term}(fluents), coeffs)
end

MetricHeuristic(metric, fluents) =
    MetricHeuristic(metric, fluents, ones(Float32, length(fluents)))

is_precomputed(h::MetricHeuristic) = isdefined(h, :goalvals)

function precompute!(h::MetricHeuristic,
                     domain::Domain, state::State, spec::Specification)
    goals = get_goal_terms(spec)
    h.goalvals = map(goals) do g
        @assert g.name == :(==) "Goal $g is not an equality constraint."
        return Float32(g.args[2].name)
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

"Computes Manhattan distance to the goal for the specified numeric fluents."
const ManhattanHeuristic = MetricHeuristic{typeof(norm1)}

ManhattanHeuristic(fluents, args...) =
    MetricHeuristic(norm1, fluents, args...)

"Computes Euclidean distance to the goal for the specified numeric fluents."
const EuclideanHeuristic = MetricHeuristic{typeof(norm2)}

EuclideanHeuristic(fluents, args...) =
    MetricHeuristic(norm2, fluents, args...)
