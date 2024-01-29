export LMLocalSmartPlanner

"""
    LMLocalPlanner(;
        lm_graph::LandmarkGraph,
        p_graph::PlanningGraph,
        internal_planner::Planner,
        max_time::Float64 = Inf,
        max_mem::Float64 = Inf
    )

Extension on LM Local. Tries to combine landmarks into groups to need less copied planners.
Landmark graphs given to this planner should not contain any cycles. Due to the need for sources of the graph for each step of the algorithm.

Adds to the procedure layed out by LM Local by first computing a compatibility matrix.
This matrix can then be used to combine landmarks into larger goals.
Procedure thus lookes as follows:
    1. Get current sources of the landmark graph
    2. Create Conjunctive Goals based on compatibiliity matrix and current sources
    2. For each GoalGroup:
        1. Copy the internal planner
        2. Solve for Group
        3. Compare with saved solution, if shorter save this solution, planner
    3. Remove used landmarks from landmark graph
    4. Update Internal Planner and Solution.
    5. Repeat from step 1 untill landmark graph is empty.
    6. Solve for orginal goal.

[1] B. van Maris, "Landmarks in Planning: Using landmarks as Intermediary Goals or as a Pseudo-Heuristic",
Delft University of Technology.
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
    @unpack lm_graph, gen_data, internal_planner, max_time = planner
    internal_planner.max_time = max_time
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
            sat = !interferes(lm_graph.nodes[i].landmark, lm_graph.nodes[j].landmark, gen_data)
            # sat = PDDL.satisfy(domain, state, Compound(:and, [lm_id_to_terms[i], lm_id_to_terms[j]]))
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
        
        # Create Conjunctive Goals based on compatibility matrix and current sources
        groups::Vector{Set{Int}} = Vector()
        for i in sources
            added = false
            new_groups::Vector{Set{Int}} = Vector()
            for goal_group::Set{Int} in groups
                compat = filter(id -> compat_mat[i.id, id], goal_group)
                # If the matching group stays the same size, new source can be added to it.
                if length(compat) == length(goal_group) 
                    added = true
                    push!(goal_group, i.id)
                # New source is compatible with something, create new group for that
                elseif length(compat) > 0
                    added = true
                    push!(compat, i.id)
                    is_new = true
                    for new_group in new_groups if issetequal(compat, new_group) is_new = false end end
                    if is_new push!(new_groups, compat) end
                end
            end

            if !added push!(groups, Set([i.id])) end
            append!(groups, new_groups)
        end
        # Turn Goal Groups in to Goal Vectors
        goal_terms::Vector{Vector{Term}} = map(group -> map(id -> lm_id_to_terms[id], collect(group)), groups)

        # For each next up Goal compute plan to get there, take shortest and add to final solution
        shortest_sol = nothing
        used_planner = nothing
        for goal in goal_terms
            # Copy planner so we dont get side effects
            copy_planner = deepcopy(internal_planner)
            copy_planner.max_time = planner.max_time - (time() - start_time)
            sub_sol = deepcopy(sol)
            inter_spec = Specification(goal)
            sub_sol.status = :in_progress
            sub_sol = search!(sub_sol, copy_planner, domain, inter_spec)
            # If we hit a timeout return the solution we are currently on.
            if sub_sol.status == :max_time
                return sub_sol
            end
            # Update used/Shortest Solution.
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

