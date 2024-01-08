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
    # Max timout in seconds
    max_time::Float64 = Inf
end

function LMLocalPlanner(lm_graph::LandmarkGraph, p_graph::PlanningGraph, internal_planner::Planner, max_time::Float32)
    return LMLocalPlanner(lm_graph, p_graph, internal_planner, max_time)
end

function solve(planner::LMLocalPlanner,
                domain::Domain, state::State, spec::Specification)
    @unpack lm_graph, p_graph, internal_planner = planner
    @unpack h_mult, heuristic, save_search = internal_planner
    saved_lm_graph = deepcopy(lm_graph)
    # Since REASONABLE and NATURAL edges often cause cycles we remove them to prevent issues.
    remove_reasonable_and_natural_edges(lm_graph)
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

    start_time = time()
    while (length(lm_graph.nodes) > 0)
        if time() - start_time >= planner.max_time
            sol.status = :max_time # Time budget reached
            return sol
        end

        sources = get_sources(lm_graph)
        if (length(sources) == 0) break end
        # For each next up LM compute plan to get there, take shortest and add to final solution
        shortest_sol = nothing
        used_lm = nothing
        used_planner = nothing
        # TODO evaluate NECESSARY edges first and only those if they exist (Speed up performance)
        for lm in sources
            # Copy planner so we dont get side effects
            copy_planner = deepcopy(internal_planner)
            sub_sol = deepcopy(sol)
            inter_spec = Specification(landmark_to_terms(lm.landmark, p_graph))
            sub_sol.status = :in_progress
            sub_sol = search!(sub_sol, copy_planner, domain, inter_spec)
            if isnothing(shortest_sol) 
                shortest_sol = sub_sol 
                used_lm = lm
                used_planner = copy_planner
            elseif length(sub_sol.trajectory) < length(shortest_sol.trajectory)
                shortest_sol = sub_sol
                used_lm = lm
                used_planner = copy_planner
            end
        end
        landmark_graph_remove_occurences(lm_graph, used_lm)    
        # Update internal_planner and sol
        internal_planner = used_planner
        sol = shortest_sol
    end

    sol = search!(sol, internal_planner, domain, spec)

    # Reset internal LM Graph to prevent not using landmarks in subsequent runs
    planner.lm_graph = saved_lm_graph

    # Return solution
    if save_search
        return sol
    elseif sol.status == :failure
        return NullSolution(sol.status)
    else
        return PathSearchSolution(sol.status, sol.plan, sol.trajectory)
    end
end

function remove_reasonable_and_natural_edges(lm_graph::LandmarkGraph)
    for lm in lm_graph.nodes
        for (child, edge) in lm.children
            if edge == REASONABLE || edge == NATURAL delete!(lm.children, child) end
        end
        for (parent, edge) in lm.parents
            if edge == REASONABLE || edge == NATURAL delete!(lm.parents, parent) end
        end
    end
end

function get_sources(lm_graph::LandmarkGraph) :: Set{LandmarkNode}
    res::Set{LandmarkNode} = Set()
    for lm in lm_graph.nodes
        if length(lm.parents) == 0 push!(res, lm) end
    end
    return res
end

function landmark_to_terms(lm::Landmark, p_graph::PlanningGraph) :: AbstractVector{<:Term}
    res::Vector{Term} = Vector()
    for fact_p :: FactPair in lm.facts
        if fact_p.value == 1
            push!(res, p_graph.conditions[fact_p.var])
        end
    end
    return res
end