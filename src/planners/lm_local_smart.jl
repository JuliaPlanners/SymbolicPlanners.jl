export LMLocalSmartPlanner

"""
"""

@kwdef mutable struct LMLocalSmartPlanner <: Planner
    # Landmark Graph that is used to generate intermediary goals
    lm_graph::LandmarkGraph
    # Planning graph to match the LM graph
    gen_data::LandmarkGenerationData
    # Planner used to solve the problem
    internal_planner::Planner
    # Max timout in seconds
    max_time::Float64 = Inf
    # Max memory in bytes
    max_mem::Float64 = Inf
end

function LMLocalSmartPlanner(lm_graph::LandmarkGraph, gen_data::LandmarkGenerationData, internal_planner::Planner, max_time::Float64)
    return LMLocalSmartPlanner(lm_graph, gen_data, internal_planner, max_time, Inf)
end

function solve(planner::LMLocalSmartPlanner,
                domain::Domain, state::State, spec::Specification)
    @unpack lm_graph, gen_data, internal_planner = planner
    @unpack h_mult, heuristic, save_search = internal_planner
    p_graph = gen_data.planning_graph
    saved_lm_graph = deepcopy(lm_graph)

    # Generate Terms for each Landmark
    lm_id_to_terms::Dict{Int, Term} = Dict()
    for (idx, lm) in enumerate(lm_graph.nodes)
        lm.id = idx
        term = landmark_to_terms(lm.landmark, p_graph)
        lm_id_to_terms[lm.id] = term
    end
    
    # Create compatibility Matrix for all terms/landmarks
    nr_nodes = length(lm_graph.nodes)
    compat_mat = trues(nr_nodes, nr_nodes)
    for i in 1:nr_nodes
        for j in i+1:nr_nodes
            # sat = !interferes(lm_graph.nodes[i].landmark, lm_graph.nodes[j].landmark, gen_data)
            sat = PDDL.satisfy(domain, state, Compound(:and, [lm_id_to_terms[i], lm_id_to_terms[j]]))
            compat_mat[i,j] = sat
            compat_mat[j,i] = sat
        end
    end
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
        if time() - start_time >= planner.max_time || gc_live_bytes() > planner.max_mem
            sol.status = :max_time # Time budget reached
            return sol
        end
        
        sources = get_sources(lm_graph)
        if (length(sources) == 0) 
            println("No new sources")
            break 
        end

        # Create Conjunctive Goals based on compatibiliity matrix and current sources
        used::Set{Int} = Set()
        goal_terms::Vector{Vector{Term}} = Vector()
        for i in sources
            if i.id in used continue end
            goal = Vector()
            for j in sources
                if compat_mat[i.id,j.id]
                    push!(goal, lm_id_to_terms[j.id])
                    push!(used, j.id)
                end
            end
            push!(goal_terms, goal)
        end

        # For each next up Goal compute plan to get there, take shortest and add to final solution
        shortest_sol = nothing
        used_planner = nothing
        for goal in goal_terms
            # Copy planner so we dont get side effects
            copy_planner = deepcopy(internal_planner)
            sub_sol = deepcopy(sol)
            inter_spec = Specification(goal)
            sub_sol.status = :in_progress
            sub_sol = search!(sub_sol, copy_planner, domain, inter_spec)
            if isnothing(shortest_sol) 
                shortest_sol = sub_sol 
                used_planner = copy_planner
            elseif length(sub_sol.trajectory) < length(shortest_sol.trajectory)
                shortest_sol = sub_sol
                used_planner = copy_planner
            end
        end
        # Update internal_planner and sol
        internal_planner = used_planner
        sol = shortest_sol
        # Find LM that was solved and remove it from LM graph
        for lm in sources
            if is_goal(Specification(lm_id_to_terms[lm.id]), domain, sol.trajectory[end])
                landmark_graph_remove_occurences(lm_graph, lm)
                landmark_graph_remove_node(lm_graph, lm)
            end
        end
    end
    sol.status = :in_progress
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

