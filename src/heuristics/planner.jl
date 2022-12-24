export PlannerHeuristic

"Computes distance to the goal based on planner solution."
@kwdef struct PlannerHeuristic{P <: Planner, DT, ST} <: Heuristic
    planner::P
    d_transform::DT = identity # Optional domain transform
    s_transform::ST = identity # Optional state transform
end

function PlannerHeuristic(
    planner::Planner;
    domain = nothing,
    state = nothing,
    d_transform = isnothing(domain) ? identity : _ -> domain,
    s_transform = isnothing(state) ? identity : _ -> state
)
    return PlannerHeuristic(;planner=planner, d_transform=d_transform,
                            s_transform=s_transform)
end

function compute(h::PlannerHeuristic,
                 domain::Domain, state::State, spec::Specification)
    domain = h.d_transform(domain)
    state = h.s_transform(state)
    sol = h.planner(domain, state, spec)
    if sol isa OrderedSolution
        return length(sol)
    elseif sol isa PolicySolution
        return -get_value(sol, state)
    else
        return Inf
    end
end
