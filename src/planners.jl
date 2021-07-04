## Abstract interface for planners and solutions ##
export Planner

"Abstract planner type, which defines the interface for planners."
abstract type Planner end

(planner::Planner)(domain::Domain, state::State, spec) =
    solve(planner, domain, state, spec)

solve(planner::Planner, domain::Domain, state::State, spec::Specification) =
    error("Not implemented.")
solve(planner::Planner, domain::Domain, state::State, goals::Vector{<:Term}) =
    solve(planner, domain, state, Specification(goals))
solve(planner::Planner, domain::Domain, state::State, goal::Term) =
    solve(planner, domain, state, Specification(goal))

# Path-search planning algorithms
include("planners/path_search.jl")
include("planners/bfs.jl")
include("planners/forward.jl")
include("planners/backward.jl")

# Policy-based planning algorithms
include("planners/rtdp.jl")
include("planners/mcts.jl")

# External planners
include("planners/external.jl")
