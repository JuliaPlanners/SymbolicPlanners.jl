## Abstract interface for planners and solutions ##
export Planner, refine, refine!

"""
    $(TYPEDEF)

Abstract planner type. Once constructed, a `planner` can be run on
a `domain`, initial `state`, and a `spec`, returning a [`Solution`](@ref).
A `problem` can be provided instead of a `state` and `spec`:

    planner(domain::Domain, state::State, spec)
    planner(domain::Domain, problem::Problem)

New planners should define [`solve`](@ref) and (optionally) [`refine!`](@ref).
"""
abstract type Planner end

(planner::Planner)(domain::Domain, state::State, spec) =
    solve(planner, domain, state, spec)
(planner::Planner)(domain::Domain, problem::Problem) =
    solve(planner, domain, problem)

"""
    solve(planner::Planner, domain::Domain, state::State, spec::Specification)

Solve a planning problem using the specified `planner`.

New subtypes of [`Planner`](@ref) should implement this method.
"""
solve(planner::Planner, domain::Domain, state::State, spec::Specification) =
    error("Not implemented.")
solve(planner::Planner, domain::Domain, state::State, goals) =
    solve(planner, domain, state, Specification(goals))
solve(planner::Planner, domain::Domain, problem::Problem) =
    solve(planner, domain, initstate(domain, problem), Specification(problem))

"""
    refine!(sol::Solution, planner::Planner, domain::Domain, state::State, spec)

Refine an existing solution (`sol`) to a planning problem (in-place).
The `spec` argument can be provided as a `Specification` or as one or more
goal `Term`s to be satisfied.
"""
refine!(sol::Solution, planner::Planner, domain::Domain, state::State, spec::Specification) =
    error("Not implemented.")
refine!(sol::Solution, planner::Planner, domain::Domain, state::State, goals) =
    refine!(sol, planner, domain, state, Specification(goals))

"""
    refine(sol::Solution, planner::Planner, domain::Domain, state::State, spec)

Refine a copy of an existing solution to a planning problem.
"""
refine(sol::Solution, planner::Planner, domain::Domain, state::State, spec::Specification) =
    refine!(copy(sol), planner, domain, state, spec)
refine(sol::Solution, planner::Planner, domain::Domain, state::State, goals) =
    refine!(copy(sol), planner, domain, state, Specification(goals))

"""
    LoggerCallback(loglevel::LogLevel = Info; options...)

A callback function for logging planner progress. The `loglevel` determines
the verbosity of the logging (default = `Info``), and `options` can be
specified to configure what is logged and how often.
"""
struct LoggerCallback <: Function
    loglevel::Logging.LogLevel
    options::Dict{Symbol, Any}
end

LoggerCallback(loglevel::Logging.LogLevel = Logging.Info; options...) =
    LoggerCallback(loglevel, Dict{Symbol, Any}(options))

# Path-search planning algorithms
include("path_search.jl")
include("reusable_tree.jl")
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
