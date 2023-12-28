

export LandmarkStatusManager

mutable struct LandmarkStatusManager
    lm_graph::LandmarkGraph
    past::Set{LandmarkNode} = nothing
    future::Set{LandmarkNode} = nothing
    curr_true::Set{LandmarkNode} = nothing
end

function get_past_landmarks(lm_status_manager::LandmarkStatusManager) :: Set{LandmarkNode} 
    return lm_status_manager.past
end

function get_future_landmarks(lm_status_manager::LandmarkStatusManager) :: Set{LandmarkNode} 
    return lm_status_manager.future
end

function get_curr_true_landmarks(lm_status_manager::LandmarkStatusManager) :: Set{LandmarkNode}
    return lm_status_manager.curr_true
end

function progress(lm_status_manager::LandmarkStatusManager, prev::State, curr::State)
    #= If previous state is nothing then we are progessing the inital state.
     Landmarks in that are part of the initial state should be added to past
     and all their children should be added to Future.
     If landmarks that are true in the initial state have parents also add that landmark to future=#
    if (isnothing(prev))
        for lm in lm_status_manager.lm_graph.nodes
            if landmark_is_true_in_state(lm.landmark, curr)
                push!(past, lm)

                for (child,edge) in lm.children
                    if (edge > 1)
                        push!(future, child)
                    end
                end
                
                # Find one parent with edge that is eith NECESSARY or GREEDY_NECESSARY
                for (parent, edge) in lm.parents
                    if edge > 1
                        push!(future, lm)
                        break
                    end
                end
            end
        end
    else
        # If the state hasn't changed because there is some do nothing action there is no need to update LM statuses
        if (prev == curr) return end
        
        for lm in lm_status_manager.lm_graph.nodes
            if landmark_is_true_in_state(lm.landmark, curr)
                # Add landmark to past if it is true in the previous state
                if lm ∉ past push!(past, lm) end
                

                # Add each child with edge that is NECESSARY or GREEDY_NECESSARY to future
                for (child, edge) in lm.children
                    if edge > 1
                        push!(future, child)
                    end
                end

                
                # Find parent that has edge NECESSARY or GREEDY_NECESSARY that is not in the past
                # Add landmark to currently true set, if it does need to remain in the future (This extra set is needed based on the definition
                # given in the paper Landmarks Revisited by Richter et al.)
                # If that parent does not exist remove this landmark from future aswell
                for (parent, edge) in lm.parents
                    if parent in future 
                        push!(curr_true, lm)
                        return
                    end
                    if parent ∉ past 
                        if edge > 1 
                            push!(curr_true, lm)
                            return
                        end
                    end
                end
                delete!(future, lm)
            elseif lm in curr_true
                # Landmark is not currently true anymore so remove from currently true set.
                delete!(curr_true, lm)
            end
        end

    end

end