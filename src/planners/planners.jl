## Abstract interface for planners and solutions ##
export Planner
export Solution, NullSolution

"Abstract planner type, which defines the interface for planners."
abstract type Planner end

(planner::Planner)(domain::Domain, state::State, goal_spec) =
    solve(planner, domain, state, goal_spec)

solve(planner::Planner, domain::Domain, state::State, spec::Specification) =
    error("Not implemented.")
solve(planner::Planner, domain::Domain, state::State, goals::Vector{<:Term}) =
    solve(planner, domain, state, GoalSpec(goals))
solve(planner::Planner, domain::Domain, state::State, goal::Term) =
    solve(planner, domain, state, GoalSpec(goal))

"Abstract solution type, which defines the interface for planner solutions."
abstract type Solution end

"Null solution that indicates no plan was found."
struct NullSolution <: Solution end

# Planners and solution types for ordered plans
include("ordered.jl")
include("bfs.jl")
include("forward.jl")
include("backward.jl")

# Policy-based planners
include("policies.jl")
include("rtdp.jl")
include("mcts.jl")

# External planners
include("external.jl")
