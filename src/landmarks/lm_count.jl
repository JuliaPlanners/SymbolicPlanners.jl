export LMCount

@kwdef mutable struct LMCount <: Heuristic
    lm_graph::LandmarkGraph
    lm_status_manager::LandmarkStatusManager
    prev_state = nothing
end

function LMCount(lm_graph::LandmarkGraph)
    return LMCount(lm_graph, LandmarkStatusManager(lm_graph), nothing)
end

function compute(h::LMCount,
                domain::Domain, state::State, spec::Specification)
    # Progress the Status Manager to update past and Future
    progress(h.lm_status_manager, h.prev_state, state)
    future = get_future_landmarks(lm_status_manager)
    past = get_past_landmarks(lm_status_manager)
    curr_true = get_curr_true_landmarks(lm_status_manager)

    # n = Number of all landmarks found
    n = length(h.lm_graph.nodes)

    # m = Number of nodes that are in lm_status_manager.past 
    m = length(past)
    
    # k = Number of nodes that are in lm_status_manager.past and also in lm_status_manager.future
    # Without being in currently true 
    future_past = intersect(future, past)
    k = length(settdiff(future_past, curr_true))

    #Update prev_state so we can update lm_status_manager in next iteration
    h.prev_state = state
    #return H value
    return n - m + k
end

is_precomputed(h::LMCount) = isdefined(h, :lm_status_manager) & isdefined(h, :lm_graph)

function precompute(h::LMCount,
                    domain::Domain, state::State)
    h.lm_status_manager = LandmarkStatusManager(h.lm_graph)
end

