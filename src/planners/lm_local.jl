

"""
"""

@kwdef mutable struct LMLocalPlanner{T <: Union{Nothing, Float64}}  <: Planner
    # Landmark Graph that is used to generate intermediary goals
    lm_graph::LandmarkGraph
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
    @unpack lm_graph, heuristic, kwargs... = planner
    # Set up the planner that will take us from goal to goal, TODO make this dynamic??
    a_star_planner = AStarPlanner(heuristic, kwargs...)

    # Extract Next up Landmarks from Starting state
    next_lms::Set{LandmarkNode} = get_starting_landmarks(planner, state)
    sol = nothing
    while (length(next_lms) > 0)
        # For each next up LM compute plan to get there, take shortest and add to final solution
        shortest_sol = nothing
        used_lm = nothing
        # TODO figure out conversion from LandmarkNode to GoalTerm
        for lm in next_lms
            inter_spec = Specification(lm)
            sub_sol = solve(a_star_planner, domain, state, inter_spec)
            if isnothing(shortest_sol) 
                shortest_sol = sub_sol 
                used_lm = lm
            end
            if length(sub_sol.plan) < length(shortest_sol.plan)
                shortest_sol = sub_sol
                used_lm = lm
            end
        end
        # Check if main goal was reached, if so return solution
        
        # Remove used LM from set and add its children to the next round
        delete!(next_lms, used_lm)
        for (child, edge) in used_lm.children
            if edge > 1 push!(next_lms, child) end
        end
        # Merge new shortest path to next subgoal with main solution

        
    end

    return sol
end

function get_starting_landmarks(planner::LMLocalPlanner, state::State) :: Set{LandmarkNode}
    res::Set{LandmarkNode}
    for lm in planner.lm_graph.nodes
        if landmark_is_true_in_state(lm.landmark, state)
            for (child,edge) in lm.children
                if (edge > 1)
                    push!(res, child)
                end
            end
        end
    end
    return res
end