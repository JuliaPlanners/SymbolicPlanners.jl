

export LandmarkStatusManager

@kwdef mutable struct LandmarkStatusManager
    lm_graph::LandmarkGraph
    p_graph::PlanningGraph
    past::Dict{State, Set{Int}}
    future::Dict{State, Set{Int}}
end

function LandmarkStatusManager(lm_graph::LandmarkGraph, p_graph::PlanningGraph)
    return LandmarkStatusManager(lm_graph, p_graph, Dict(), Dict())
end

function get_past_landmarks(lm_status_manager::LandmarkStatusManager, state::State) :: Set{Int} 
    if !haskey(lm_status_manager.past, state)
        lm_status_manager.past[state] = Set(collect(1:length(lm_status_manager.lm_graph.nodes)))
    end
    return lm_status_manager.past[state]
end

function get_future_landmarks(lm_status_manager::LandmarkStatusManager, state::State) :: Set{Int} 
    if !haskey(lm_status_manager.future, state)
        lm_status_manager.future[state] = Set()
    end
    return lm_status_manager.future[state]
end

function progress_initial_state(lm_status_manager::LandmarkStatusManager, initial_state::State)
    @unpack lm_graph, p_graph = lm_status_manager

    past = get_past_landmarks(lm_status_manager, initial_state)
    future = get_future_landmarks(lm_status_manager, initial_state)   
    not_past::Set{Int} = Set()
    for lm in lm_graph.nodes
        if landmark_is_true_in_state(lm.landmark, p_graph, initial_state)
            # If there is one parent that does not hold in the initial state and it comes before this landmark we can put this landmark in future.
            for (parent, edge) in lm.parents
                if (parent.id ∉ past && !landmark_is_true_in_state(parent.landmark, p_graph, initial_state)) && (edge != REASONABLE)
                    push!(future, lm.id)
                    break
                end
            end
        else
            # If the Landmark is not true now it must become true at some point.
            push!(not_past, lm.id)
            push!(future, lm.id)
        end
    end
    setdiff!(past, not_past)
end

function progress(lm_status_manager::LandmarkStatusManager, prev::State, curr::State)
    # If the state hasn't changed because there is some do nothing action there is no need to update LM statuses
    if (prev == curr) return end
    @unpack lm_graph, p_graph = lm_status_manager

    parent_past::Set{Int} = get_past_landmarks(lm_status_manager, prev)
    past::Set{Int} = get_past_landmarks(lm_status_manager, curr)
    parent_future::Set{Int} = get_future_landmarks(lm_status_manager, prev)
    future::Set{Int} = get_future_landmarks(lm_status_manager, curr)   

    not_past::Set{Int} = Set()
    for lm in lm_graph.nodes
        if lm.id in parent_future
            #If landmark was in the future and its still not true it remains in the future
            #If landmark not true in the parent_past and not true in this state, not in past
            if (!landmark_is_true_in_state(lm.landmark, p_graph, curr))
                push!(future, lm.id)
                if lm.id ∉ parent_past push!(not_past, lm.id) end
            #If Landmark not true in the previous state, it should be added to future again because it is also not true in this state.
            elseif (landmark_is_true_in_state(lm.landmark, p_graph, prev))
                push!(future, lm.id)
            end
        end
        #Goal LM's always in future
        if lm.landmark.is_true_in_goal push!(future, lm.id) end

        # Greedy Necessary orderings
        for (child, edge) in lm.children
            if (edge == NECESSARY || edge == GREEDY_NECESSARY) && (child.id ∉ past) && !landmark_is_true_in_state(lm.landmark, p_graph, curr)
                push!(future, lm.id)
            end
        end
        # Reasonable orderings
        for (parent, edge) in lm.parents
            if (edge == REASONABLE) && (parent.id ∉ past)
                push!(future, lm.id)
            end
        end
    end
    setdiff!(past, not_past)
end