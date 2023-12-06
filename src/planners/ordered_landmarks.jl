export OrderedLandmarksPlanner

@kwdef mutable struct OrderedLandmarksPlanner <: Planner
    "Maximum number of search nodes before termination."
    max_nodes::Int = typemax(Int)
    "Maximum time in seconds before planner times out."
    max_time::Float64 = Inf
    "Flag to print debug information during search."
    verbose::Bool = false
    "Callback function for logging, etc."
    callback::Union{Nothing, Function} = verbose ? LoggerCallback() : nothing
end

@auto_hash OrderedLandmarksPlanner
@auto_equals OrderedLandmarksPlanner

function Base.copy(p::OrderedLandmarksPlanner)
    return OrderedLandmarksPlanner(p.max_nodes, p.max_time, p.verbose, p.callback)
end

function solve(planner::OrderedLandmarksPlanner, domain::Domain, state::State, spec::Specification)
    f = no_fact()
    # Placeholder
    f_planner = AStarPlanner(HAdd(), save_search=true)
    return solve(f_planner, domain, state, spec)
end

function (cb::LoggerCallback)(
    planner::OrderedLandmarksPlanner,
    sol::PathSearchSolution, node_id::UInt, priority
)
    # TODO
    return nothing
end
