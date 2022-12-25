## Abstract interface for planners and solutions ##
export Planner, refine, refine!

"Abstract planner type, which defines the interface for planners."
abstract type Planner end

(planner::Planner)(domain::Domain, state::State, spec) =
    solve(planner, domain, state, spec)
(planner::Planner)(domain::Domain, problem::Problem) =
    solve(planner, domain, problem)

"Solve a planning problem using the specified `planner`."
solve(planner::Planner, domain::Domain, state::State, spec::Specification) =
    error("Not implemented.")
solve(planner::Planner, domain::Domain, state::State, goals) =
    solve(planner, domain, state, Specification(goals))
solve(planner::Planner, domain::Domain, problem::Problem) =
    solve(planner, domain, initstate(domain, problem), Specification(problem))

"Refine an existing solution to a planning problem (in-place)."
refine!(sol::Solution, planner::Planner, domain::Domain, state::State, spec::Specification) =
    error("Not implemented.")
refine!(sol::Solution, planner::Planner, domain::Domain, state::State, goals) =
    refine!(sol, planner, domain, state, Specification(goals))

"Refine a copy of an existing solution to a planning problem."
refine(sol::Solution, planner::Planner, domain::Domain, state::State, spec::Specification) =
    refine!(copy(sol), planner, domain, state, spec)
refine(sol::Solution, planner::Planner, domain::Domain, state::State, goals) =
    refine!(copy(sol), planner, domain, state, Specification(goals))

# Path-search planning algorithms
include("path_search.jl")
include("bfs.jl")
include("forward.jl")
include("backward.jl")
include("bidirectional.jl")

# Policy-based planning algorithms
include("rtdp.jl")
include("rths.jl")
include("mcts.jl")

# External planners
include("external.jl")
