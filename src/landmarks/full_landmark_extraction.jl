export full_landmark_extraction

function full_landmark_extraction(domain::Domain, problem::Problem)
    
    # Initialize state and specification
    start_state = initstate(domain, problem)
    spec = Specification(problem)

    # Compute landmark graph
    (landmark_graph, generation_data)= compute_landmark_graph(domain, start_state, spec)
    
    # Extract landmarks from landmark graph
    landmarks = Set{Landmark}()

    for landmark_node in landmark_graph.nodes
        push!(landmarks, landmark_node.landmark)    
    end

    # println("Amount of landmarks found in landmark graph: ", length(landmarks))

    planning_graph = generation_data.planning_graph

    # Propagate landmarks
    zhu_landmarks = propagate_landmarks(planning_graph, domain, start_state)

    # println("Amount of landmarks found in propagation: ", length(propagated_landmarks))

    # Merge initial and propagated landmarks
    merged_landmarks = merge_landmarks(landmarks, zhu_landmarks)

    # println("Amount of landmarks found after merging: ", length(landmarks))


    # VERIFICATION IS NOW DONE WHEN GENERATING LANDMARKS

    # Verify that all nodes are actual landmarks
    # Landmarks are verified by checking if the goals are reachable if the landmark is removed from the delete list of the actions leading to them
    # If the goals are reachable, the landmark is not a landmark and is removed from the set of landmarks
    # If the goals are not reachable, the landmark is a landmark and is added to the set of landmarks
    # verified_landmarks = verify_landmarks(landmarks, planning_graph, domain, problem)


    # Compute disjunctive landmarks using the verified landmarks

    # disjuctive_landmarks = compute_disjunctive_landmarks(verified_landmarks, domain, problem)

    return (merged_landmarks, landmarks, zhu_landmarks, generation_data)
    
end



function propagate_landmarks(planning_graph::PlanningGraph, domain::Domain, state::State)

    zhu_landmarks = graph_label(planning_graph, domain, state)

    propagated_landmarks = Set{Landmark}()
    
    for landmark in zhu_landmarks
        current_facts = Vector{FactPair}()
        for fact in landmark.labels
            push!(current_facts, FactPair(fact, 1))
        end
        push!(propagated_landmarks, Landmark(current_facts, (length(current_facts) > 1 ? true : false), false, true, false, Set{}(), Set{}()))
    end

    return propagated_landmarks
end

function merge_landmarks(landmarks::Set{Landmark}, propagated_landmarks::Set{Landmark})

    merged_facts = Set{Vector{FactPair}}()

    merged_landmarks = Set{Landmark}()

    for landmark in landmarks
        push!(merged_facts, landmark.facts)
        push!(merged_landmarks, landmark)
    end

    for landmark in propagated_landmarks
        if landmark.facts in merged_facts
            continue
        else
            push!(merged_facts, landmark.facts)
            push!(merged_landmarks, landmark)
        end
    end

    return merged_landmarks
end



function verify_landmarks(landmarks::Set{Landmark}, planning_graph::PlanningGraph, domain::Domain, problem::Problem)

    # Initialize state and specification
    
    spec = Specification(problem)

    # Initialize planner
    planner = AStarPlanner(HAdd(), save_search=true)

    initial_domain = domain

    for landmark in landmarks
        # Remove preconditions of landmark from the delete list of the total action list.
        # This is done by creating a new domain with the modified delete lists
        domain = modify_domain(domain, landmark, planning_graph)

        state = initstate(domain, problem)

        # Run planner
        sol = planner(domain, state, spec)

        # If no solution is found then it is a landmark
        if sol.status != -1
            continue
        else
            # A solution was found, meaning current landmark is not a landmark in the domain
            delete!(landmarks, landmark)
        end

        domain = initial_domain

    end

    return landmarks

end



function modify_domain(domain::Domain, landmark::Landmark, planning_graph::PlanningGraph)

    
    new_actions = Set{Symbol}()

    # Find all actions that dont add the landmark

    for fact in landmark.facts
        # Retrieve the name of the action e.g. "stack"
        curr_action = planning_graph.conditions[fact.var].name
        push!(new_actions, curr_action)
    end


    # Go over add list of actions. If the landmark is in the add list, remove the delete list from that action
    for curr_action in new_actions

        # Go over all actions in the domain
        for action in domain.actions
            # Get name of actions in actions add list
            action_names = map(x -> x.name, action.second.effect.args)
            if curr_action in action_names
                continue 
            else 
                filter!(x -> x.name != Symbol("not"), action.second.effect.args)
            end
        end
    end


    return domain

end
