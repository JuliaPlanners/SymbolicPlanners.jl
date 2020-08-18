## Abstract interface for planners ##
export Planner
export set_max_resource

"Abstract planner type, which defines the interface for planners."
abstract type Planner end

(planner::Planner)(domain::Domain, state::State, goal_spec) =
    call(planner, domain, state, goal_spec)

call(planner::Planner, domain::Domain, state::State, goal::GoalSpec) =
    error("Not implemented.")
call(planner::Planner, domain::Domain, state::State, goals::Vector{<:Term}) =
    call(planner, domain, state, GoalSpec(goals))
call(planner::Planner, domain::Domain, state::State, goal::Term) =
    call(planner, domain, state, GoalSpec(goal))

"Return copy of the planner with adjusted resource bound."
set_max_resource(planner::Planner, val) = planner

include("common.jl")
include("bfs.jl")
include("forward.jl")
include("backward.jl")
include("external.jl")
