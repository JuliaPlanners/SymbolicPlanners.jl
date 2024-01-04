export LMLocalPlanner

"""
"""

@kwdef mutable struct LMLocalPlanner <: Planner
    # Landmark Graph that is used to generate intermediary goals
    lm_graph::LandmarkGraph
    # Planning graph to match the LM graph
    p_graph::PlanningGraph
    # Planner used to solve the problem
    internal_planner::Planner
end

function solve(planner::LMLocalPlanner,
                domain::Domain, state::State, spec::Specification)
    @unpack lm_graph, p_graph, internal_planner = planner

    # Extract Next up Landmarks from Starting state
    next_lms::Set{LandmarkNode} = get_starting_landmarks(lm_graph, p_graph, state)

    @unpack h_mult, heuristic, save_search = internal_planner
    # Simplify goal specification
    spec = simplify_goal(spec, domain, state)
    # Precompute heuristic information
    precompute!(heuristic, domain, state, spec)
    # Initialize search tree and priority queue
    node_id = hash(state)
    search_tree = Dict(node_id => PathNode(node_id, state, 0.0))
    est_cost::Float32 = h_mult * compute(heuristic, domain, state, spec)
    priority = (est_cost, est_cost, 0)
    queue = PriorityQueue(node_id => priority)
    search_order = UInt[]
    sol = PathSearchSolution(:in_progress, Term[], Vector{typeof(state)}(), 0, search_tree, queue, search_order)


    while (length(lm_graph.nodes) > 0)
        if (length(next_lms) == 0) break end
        # For each next up LM compute plan to get there, take shortest and add to final solution
        shortest_sol = nothing
        used_lm = nothing
        used_planner = nothing
        # TODO evaluate NECESSARY edges first and only those if they exist (Speed up performance)
        for lm in next_lms
            # Copy planner so we dont get side effects
            copy_planner = deepcopy(internal_planner)
            sub_sol = deepcopy(sol)
            inter_spec = Specification(landmark_to_terms(lm, p_graph))
            sub_sol = search!(sub_sol, copy_planner, domain, inter_spec)
            if isnothing(shortest_sol) 
                shortest_sol = sub_sol 
                used_lm = lm
                used_planner = copy_planner
            elseif length(sub_sol.plan) < length(shortest_sol.plan)
                shortest_sol = sub_sol
                used_lm = lm
                used_planner = copy_planner
            end
        end
        # Remove used LM from set and add its children to the next round
        delete!(next_lms, used_lm)
        for (child, edge) in used_lm.children
            if edge > 1 push!(next_lms, child) end
        end
        landmark_graph_remove_occurences(lm_graph, used_lm)    
        # Update internal_planner and sol
        internal_planner = used_planner
        sol = shortest_sol   
    end

    return search!(sol, used_planner, domain, spec)
end

function get_starting_landmarks(lm_graph::LandmarkGraph, p_graph::PlanningGraph, state::State) :: Set{LandmarkNode}
    res::Set{LandmarkNode} = Set()
    for lm in lm_graph.nodes
        if landmark_is_true_in_state(lm.landmark, p_graph, state)
            for (child,edge) in lm.children
                if (edge > 1)
                    push!(res, child)
                end
            end
        end
    end
    return res
end

function landmark_to_terms(lm::Landmark, p_graph::PlanningGraph) :: AbstractVector{<:Term}
    res = Vector{<:Term}()
    for fact_p :: FactPair in lm.facts
        if fact_p.value == 1
            push!(res, p_graph.conditions[fact.var])
        end
    end
    return res
end