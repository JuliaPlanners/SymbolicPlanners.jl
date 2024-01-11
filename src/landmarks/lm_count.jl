export LMCount

@kwdef mutable struct LMCount <: Heuristic
    lm_graph::LandmarkGraph
    lm_status_manager::LandmarkStatusManager
    prev_state
    is_precomputed::Bool
    nr_nodes::Int
end

function LMCount(lm_graph::LandmarkGraph, p_graph::PlanningGraph)
    return LMCount(lm_graph, LandmarkStatusManager(lm_graph, p_graph), nothing, false, 0)
end

function compute(h::LMCount,
                domain::Domain, state::State, spec::Specification)
    # Progress the Status Manager to update past and Future
    if !(h.prev_state isa State)
        println("Previous state not updated in planner!")
        return 0
    else 
        #Previous state of this Heuristic should be updated in the planner that uses it, See "Forward.jl" at line 243
        progress(h.lm_status_manager, h.prev_state, state)
        future = get_future_landmarks(h.lm_status_manager, state)
        past = get_past_landmarks(h.lm_status_manager, state)

        
        h_val = 0
        for i in 1:h.nr_nodes
            if i ∉ past h_val += 1
            elseif i in future h_val += 1 end
        end
        # # n = Number of all landmarks found
        # n = length(h.lm_graph.nodes)
        
        # # m = Number of nodes that are in lm_status_manager.past 
        # m = length(past)
            
        # # k = Number of nodes that are in lm_status_manager.past and also in lm_status_manager.future
        # future_past = intersect(future, past)
        # k = length(future_past)

        #return H value
        #return n - m + k
        return h_val
    end
end

is_precomputed(h::LMCount) = h.is_precomputed

function precompute!(h::LMCount,
                    domain::Domain, state::State, spec::Specification)
    return precompute!(h, domain, state)
end

function precompute!(h::LMCount, domain::Domain, state::State)
    progress_initial_state(h.lm_status_manager, state)
    h.prev_state = state
    h.nr_nodes = length(h.lm_graph.nodes)
    h.is_precomputed = true
    return h
end

