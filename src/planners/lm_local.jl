

"""
"""

@kwdef mutable struct LMLocalPlanner{T <: Union{Nothing, Float64}}  <: Planner
    # Landmark Graph that is used to generate intermediary goals
    lm_graph::LandmarkGraph
    # Landmark Status manager to get Sources for next "goal"
    lm_status_manager::LandmarkStatusManager
    # Search heuristic that estimates cost of a state to the goal.
    heuristic::Heuristic = HAdd()
    # Amount of Boltzmann search noise (`nothing` for deterministic search).
    search_noise::T = nothing
    # Path cost multiplier when computing the ``f`` value of a search node.
    g_mult::Float32 = 1.0f0
    # Heuristic multiplier when computing the ``f`` value of a search node.
    h_mult::Float32 = 1.0f0
    # Maximum number of search nodes before termination.
    max_nodes::Int = typemax(Int)
    # Maximum time in seconds before planner times out.
    max_time::Float64 = Int
    # Flag to terminate search if the heuristic estimates an infinite cost.
    fail_fast::Bool = false
    # Flag to save the search tree and frontier in the returned solution.
    save_search::Bool = false
    # Flag to save the node expansion order in the returned solution.
    save_search_order::Bool = save_search
    # Flag to print debug information during search.
    verbose::Bool = false
    # Callback function for logging, etc.
    callback::Union{Nothing, Function} = verbose ? LoggerCallback() : nothing
end

function solve(planner::LMLocalPlanner,
                domain::Domain, state::State, spec::Specification)
    @unpack lm_graph, lm_status_manager, heuristic, kwargs... = planner
    # Set up the planner that will take us from goal to goal, TODO make this dynamic??
    a_star_planner = AStarPlanner(heuristic, kwargs...)
    # Set up landmark status manager and initialize the first state
    lm_status_manager = LandmarkStatusManager(lm_graph)
    progress(lm_status_manager, nothing, state)

    sources::Set{LandmarkNode} = get_source_landmarks(lm_status_manager)
    sol = nothing
    while (length(sources) > 0)
        # For each Source compute plan to get there, take shortest and add to final solution
        shortest_sol = nothing
        # TODO figure out conversion from LandmarkNode to GoalTerm
        # TODO figure out how to update the LM Status manager on each action
        for source in sources
            inter_spec = Specification(source)
            sub_sol = solve(a_star_planner, domain, state, inter_spec)
            if isnothing(shortest_sol) shortest_sol = sub_sol end
            if length(sub_sol.plan) < length(shortest_sol.plan) shortest_sol = sub_sol end
        end
        # Merge new shortest path to next subgoal with main solution

        # Check if main goal was reached, if so return solution
        
    end

end