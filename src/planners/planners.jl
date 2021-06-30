## Abstract interface for planners and solutions ##
export Planner

"Abstract planner type, which defines the interface for planners."
abstract type Planner end

(planner::Planner)(domain::Domain, state::State, goal_spec) =
    solve(planner, domain, state, goal_spec)

solve(planner::Planner, domain::Domain, state::State, goal::GoalSpec) =
    error("Not implemented.")
solve(planner::Planner, domain::Domain, state::State, goals::Vector{<:Term}) =
    solve(planner, domain, state, GoalSpec(goals))
solve(planner::Planner, domain::Domain, state::State, goal::Term) =
    solve(planner, domain, state, GoalSpec(goal))

# Solution types and utilities
include("solutions.jl")

# Search-based planners
include("bfs.jl")
include("forward.jl")
include("backward.jl")

# Policy-based planners
include("rtdp.jl")

# External planners
include("external.jl")
