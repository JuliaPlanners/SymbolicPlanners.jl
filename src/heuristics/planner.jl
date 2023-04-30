export PlannerHeuristic

"""
    PlannerHeuristic(planner, [d_transform, s_transform])

Computes distance to the goal based on the solution returned by `planner`.

If an [`OrderedSolution`](@ref) is returned, the cost of solution is used
as the heuristic estimate, (plus the heuristic value of final state, if the 
planner is a [`ForwardPlanner`](@ref).)

If a [`PolicySolution`](@ref) is returned, the negated value
(returned by [`get_value`](@ref)) is used as the heuristic estimate.

If `d_transform` or `s_transform` are provided, this transforms the input
domain or state it is passed to the `planner`. This can be used to relax the 
problem (e.g. by simplifying the domain, or adding / deleting predicates in
the state).
"""
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
        actions = collect(sol)
        cost = -PDDL.simulate(RewardAccumulator(), domain, state, actions, spec)
        if h.planner isa ForwardPlanner
            inner_h = h.planner.heuristic
            cost += inner_h(domain, sol.trajectory[end], spec)
        end
        return cost
    elseif sol isa PolicySolution
        return -get_value(sol, state)
    else
        return Inf
    end
end
