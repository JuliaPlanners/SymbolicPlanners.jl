export LMCount

@kwdef mutable struct LMCount <: Heuristic
    lm_graph::LandmarkGraph
    lm_status_manager::LandmarkStatusManager
    prev_state
    is_precomputed::Bool
end

function LMCount(lm_graph::LandmarkGraph, p_graph::PlanningGraph)
    return LMCount(lm_graph, LandmarkStatusManager(lm_graph, p_graph), nothing, false)
end

function compute(h::LMCount,
                domain::Domain, state::State, spec::Specification)
    # Progress the Status Manager to update past and Future
    if !(h.prev_state isa State)
        println("Previous state not updated in planner!")
        return 0
    else 
        progress(h.lm_status_manager, h.prev_state, state)
        future = get_future_landmarks(h.lm_status_manager, state)
        past = get_past_landmarks(h.lm_status_manager, state)

        # n = Number of all landmarks found
        n = length(h.lm_graph.nodes)

        # m = Number of nodes that are in lm_status_manager.past 
        m = length(past)
    
        # k = Number of nodes that are in lm_status_manager.past and also in lm_status_manager.future
        future_past = intersect(future, past)
        k = length(future_past)
        #Previous state of this Heuristic should be updated in the planner that uses it, See "Forward.jl" at line 243
        #return H value
        
        # println("Step: ------------")
        # print_past::Set{Int} = Set()
        # print_future::Set{Int} = Set()
        # for node::Tuple{Int, LandmarkNode} in enumerate(h.lm_graph.nodes)
        #     if node[2] in past push!(print_past, node[1]) end
        #     if node[2] in future push!(print_future, node[1]) end
        # end
        # println("Past: $(print_past)")
        # println("Future: $(print_future)")
        # println("Eq: N->$n, M->$m, K->$k, Res:$(n-m+k)")
        return n - m + k
    end
end

is_precomputed(h::LMCount) = h.is_precomputed

function precompute!(h::LMCount,
                    domain::Domain, state::State, spec::Specification)
    return precompute!(h, domain, state)
end

function precompute!(h::LMCount, domain::Domain, state::State)
    for (idx, lm) in enumerate(h.lm_graph.nodes)
        lm.id = idx
    end
    progress_initial_state(h.lm_status_manager, state)
    h.prev_state = state
    h.is_precomputed = true
    return h
end

