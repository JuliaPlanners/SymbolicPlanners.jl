

"""
"""

@kwdef mutable struct LMLocalPlanner{T <: Union{Nothing, Float64}}  <: Planner
    # Landmark Graph that is used to generate intermediary goals
    lm_graph::LandmarkGraph
    # Planner used to solve the problem
    internal_planner::Planner
end

function solve(planner::LMLocalPlanner,
                domain::Domain, state::State, spec::Specification)
    @unpack lm_graph, internal_planner = planner

    # Extract Next up Landmarks from Starting state
    next_lms::Set{LandmarkNode} = get_starting_landmarks(planner, state)
    sol = nothing
    while (sol.status != :success)
        # For each next up LM compute plan to get there, take shortest and add to final solution
        shortest_sol = nothing
        used_lm = nothing
        # TODO figure out conversion from LandmarkNode to GoalTerm
        # TODO evaluate NECESSARY edges first and only those if they exist (Speed up performance)
        for lm in next_lms
            inter_spec = Specification(lm)
            sub_sol = solve(internal_planner, domain, state, inter_spec)
            if isnothing(shortest_sol) 
                shortest_sol = sub_sol 
                used_lm = lm
            elseif length(sub_sol.plan) < length(shortest_sol.plan)
                shortest_sol = sub_sol
                used_lm = lm
            end
        end
        # Check if main goal was reached, if so merge solution and set it as done


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