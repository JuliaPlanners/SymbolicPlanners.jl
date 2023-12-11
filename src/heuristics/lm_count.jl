

mutable struct LMCount <: Heuristic
    lm_graph::landmark_graph
    lm_status_manager::LandmarkGraph
    prev_state::State = nothing
end

function compute(h::LMCount,
                domain::Domain, state::State, spec::Specification)
    # Progress the Status Manager to update past and Future
    progress(lm_status_manager, prev_state, state)
    future = get_future_landmarks(lm_status_manager)
    past = get_past_landmarks(lm_status_manager)

    # n = Number of all landmarks found
    n = length(h.lm_graph.nodes)

    # m = Number of nodes that are in lm_status_manager.past but not in lm_status_manager.future
    future_past = intersect(future, past)
    clean_past = ([])
    for i in past
        if i ∉ future_past
        push!(clean_past, i)
    end
    m = length(clean_past)

    # k = Number of nodes that are in lm_status_manager.past and also in lm_status_manager.future
    # So the intersect of past and future
    k = length(future_past)

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

