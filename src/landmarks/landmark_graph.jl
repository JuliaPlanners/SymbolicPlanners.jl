import Base.==
import Base.<
import Base.hash
import DataStructures.Queue
import SymbolicPlanners.build_planning_graph

export compute_landmark_graph, LandmarkGraph, LandmarkNode, Landmark, FactPair

mutable struct FactPair
    var::Int
    value::Int
end

function no_fact() :: FactPair
    return FactPair(-1, -1)
end

function (<)(l::FactPair, r::FactPair)
    return l.var < r.var || (l.var == r.var && l.value < r.value)
end
function (==)(l::FactPair, r::FactPair)
    return l.var == r.var && l.value == r.value
end
function hash(f::FactPair, h::UInt)
    return hash(f.var, hash(f.value, hash(FactPair, h)))
end


mutable struct Landmark
    facts::Vector{FactPair}
    disjunctive::Bool
    conjunctive::Bool
    is_true_in_goal::Bool
    is_derived::Bool

    first_achievers::Set{Int}
    possible_achievers::Set{Int}
end

function landmark_is_true_in_state(landmark::Landmark, state::State) :: Bool
    if landmark.disjunctive
        for fact::FactPair in landmark.facts
            if state[fact.var].Val == fact.value
                return true
            end
        end
        return false
    else
        for fact::FactPair in landmark.facts
            if state[fact.var].Val != fact.value
                return false
            end
        end
        return true
    end
end


@enum EdgeType begin
    NECESSARY = 3
    GREEDY_NECESSARY = 2
    NATURAL = 1
    REASONABLE = 0
end


mutable struct LandmarkNode
    id::Int
    landmark::Landmark
    parents::Dict{LandmarkNode, EdgeType}
    children::Dict{LandmarkNode, EdgeType}
end

function node_from_landmark(landmark::Landmark) :: LandmarkNode
    return LandmarkNode(-1, landmark, Dict([]), Dict([]))
end


mutable struct LandmarkGraph
    num_conjunctive_landmarks::Int
    num_disjunctive_landmarks::Int
    simple_landmarks_to_nodes::Dict{FactPair, LandmarkNode}
    disjunctive_landmarks_to_nodes::Dict{FactPair, LandmarkNode}
    nodes::Vector{LandmarkNode}
end

function landmark_graph_get_num_edges(landmark_graph::LandmarkGraph) :: Int
    total = 0
    for node in landmark_graph.nodes
        total += size(node.children)
    end
    return total
end

function landmark_graph_get_node(landmark_graph::LandmarkGraph, fact_pair::FactPair) :: Union{LandmarkNode, Nothing}
    return get(landmark_graph.simple_landmarks_to_nodes, fact_pair) do
        get(landmark_graph.disjunctive_landmarks_to_nodes, fact_pair, nothing)
    end
end

function landmark_graph_overlaps_disjunctive(landmark_graph::LandmarkGraph, fact_pairs::Set{FactPair}) :: Bool
    for fact::FactPair in fact_pairs
        if haskey(landmark_graph.disjunctive_landmarks_to_nodes, fact)
            return true
        end
    end
    return false
end

function landmark_graph_contains_disjunctive(landmark_graph::LandmarkGraph, fact_pairs::Set{FactPair}) :: Bool
    node::Union{LandmarkNode, Nothing} = nothing
    for fact::FactPair in fact_pairs
        fact_node::Union{LandmarkNode, Nothing} = get(landmark_graph.disjunctive_landmarks_to_nodes, fact, nothing)
        if isnothing(fact_node)
            return false
        elseif isnothing(node) || node == fact_node
            node = fact_node
        else
            return false
        end
    end
    return true
end

function landmark_graph_contains_landmark(landmark_graph::LandmarkGraph, fact_pair::FactPair) :: Bool
    return haskey(landmark_graph.simple_landmarks_to_nodes, fact_pair) ||
        haskey(landmark_graph.disjunctive_landmarks_to_nodes, fact_pair)
end

function landmark_graph_add_landmark(landmark_graph::LandmarkGraph, landmark::Landmark) :: LandmarkNode
    new_node::LandmarkNode = node_from_landmark(landmark)
    push!(landmark_graph.nodes, new_node)
    if landmark.disjunctive
        for fact::FactPair in landmark.facts
            landmark_graph.disjunctive_landmarks_to_nodes[fact] = new_node
        end
        landmark_graph.num_disjunctive_landmarks += 1
    elseif landmark.conjunctive
        landmark.num_conjunctive_landmarks += 1
    else
        landmark_graph.simple_landmarks_to_nodes[first(landmark.facts)] = new_node
    end
    return new_node
end

function landmark_graph_remove_occurences(landmark_graph::LandmarkGraph, node::LandmarkNode)
    for parent::LandmarkNode in keys(node.parents)
        delete!(parent.children, node)
    end
    for child::LandmarkNode in keys(node.children)
        delete!(child.parents, node)
    end
    
    landmark::Landmark = node.landmark
    if landmark.disjunctive
        landmark_graph.num_disjunctive_landmarks -= 1
        for fact::FactPair in landmark.facts
            delete!(landmark_graph.disjunctive_landmarks_to_nodes, fact)
        end
    elseif landmark.conjunctive
        landmark_graph.num_conjunctive_landmarks -= 1
    else
        delete!(landmark_graph.simple_landmarks_to_nodes, first(landmark.facts))
    end
end

function landmark_graph_remove_node(landmark_graph::LandmarkGraph, node::LandmarkNode)
    filter!(n -> n != node, landmark_graph.nodes)
end

function landmark_graph_remove_node_if(landmark_graph::LandmarkGraph, condition::Function)
    filter!(node -> begin
        if condition(node)
            landmark_graph_remove_occurences(landmark_graph, node)
            return false
        end
        return true
    end, landmark_graph.nodes)
end

function landmark_graph_set_landmark_ids(landmark_graph::LandmarkGraph)
    id = 0
    for node::LandmarkNode in landmark_graph.nodes
        node.id = id
        id = id + 1
    end
end


mutable struct Proposition
    fact::FactPair
    reached::Bool
    excluded::Bool
end

function (<)(l::Proposition, r::Proposition)
    return l.fact < r.fact
end
function (==)(l::Proposition, r::Proposition)
    return l.fact == r.fact
end

function enqueue_if_necessary(queue::Queue{Proposition}, proposition::Proposition)
    if !proposition.reached
        proposition.reached = true
        enqueue!(queue, proposition)
    end
end

mutable struct LandmarkGenerationData
    planning_graph::PlanningGraph
    term_index::Dict{Term, Int}
    prop_queue::Queue{Proposition}
    excluded_operators::Set{GroundAction}
    unsatisfied_preconditions::Dict{GroundAction, Int}
    propositions::Dict{FactPair, Proposition}
end

function setup_exploration_queue(generation_data::LandmarkGenerationData, state::Vector{FactPair}, excluded_props::Vector{FactPair}, excluded_op_ids::Vector{Int})
    empty!(generation_data.prop_queue)
    generation_data.propositions = Dict(map(f -> (f, Proposition(f, false, false)), [FactPair(generation_data.term_index[x], y) for x in values(generation_data.planning_graph.conditions), y in [0, 1]]))

    for fact::FactPair in excluded_props
        generation_data.propositions[fact].excluded = true
    end

    for fact::FactPair in state
        enqueue_if_necessary(generation_data.prop_queue, generation_data.propositions[fact])
    end

    operation_index::Dict{GroundAction, Int} = Dict(map(reverse, enumerate(generation_data.planning_graph.actions)))
    op_ids_to_mark::Set{Int} = Set(excluded_op_ids)
    for action::GroundAction in generation_data.planning_graph.actions
        for effect::Term in action.effect.add
            if isempty(action.preconds) && generation_data.propositions[FactPair(generation_data.term_index[effect], 1)]
                push!(op_ids_to_mark, operation_index[action])
                break
            end
        end
    end

    generation_data.unsatisfied_preconditions = Dict()
    generation_data.excluded_operators = Set()
    for unary::GroundAction in generation_data.planning_graph.actions
        generation_data.unsatisfied_preconditions[unary] = length(unary.preconds)

        if all(map(e -> generation_data.propositions[FactPair(generation_data.term_index[e], 1)].excluded, unary.effect.add)) || operation_index[unary] in op_ids_to_mark
            push!(generation_data.excluded_operators, unary)
        else
            delete!(generation_data.excluded_operators, unary)

            if generation_data.unsatisfied_preconditions[unary] == 0
                enqueue_if_necessary(generation_data.prop_queue, generation_data.propositions[fact])
            end
        end
    end

    for fact::FactPair in excluded_props
        generation_data.propositions[fact].excluded = false
    end
end

function relaxed_exploration(generation_data::LandmarkGenerationData)
    while !isempty(generation_data.prop_queue)
        prop::Proposition = dequeue!(generation_data.prop_queue)

        triggered_operators::Vector{GroundAction} = map(i -> generation_data.planning_graph.actions[i], generation_data.planning_graph.effect_map[generation_data.planning_graph.conditions[prop.fact.var]])
        for operator::GroundAction in triggered_operators
            if !(operator in generation_data.excluded_operators)
                generation_data.unsatisfied_preconditions[operator] -= 1
                
                if generation_data.unsatisfied_preconditions[operator] == 0
                    for effect::Term in operator.effect.add
                        enqueue_if_necessary(generation_data.prop_queue, generation_data.propositions[FactPair(generation_data.term_index[effect], 1)])
                    end
                end
            end
        end
    end
end

function compute_relaxed_reachability(generation_data::LandmarkGenerationData, state::Vector{FactPair}, excluded_props::Vector{FactPair}, excluded_op_ids::Vector{Int}) :: Dict{Pair{Int, Int}, Bool}
    setup_exploration_queue(generation_data, state, excluded_props, excluded_op_ids)
    relaxed_exploration(generation_data)

    reached::Dict{Pair{Int, Int}, Bool} = Dict()
    for prop::Proposition in values(generation_data.propositions)
        reached[Pair(prop.fact.var, prop.fact.value)] = prop.reached
    end
    return reached
end

function relaxed_task_solvable(landmark::Landmark, state::Vector{FactPair}, generation_data::LandmarkGenerationData, spec::Specification) :: Bool
    excluded_op_ids::Vector{Int} = []
    excluded_props::Vector{FactPair} = landmark.facts

    reached::Dict{Pair{Int, Int}, Bool} = compute_relaxed_reachability(generation_data, state, excluded_props, excluded_op_ids)
    for fact::Term in spec.terms
        if !reached[Pair(generation_data.term_index[fact], 1)]
            return false
        end
    end

    return true
end

function is_landmark_precondition(operator::GroundAction, landmark::Landmark, generation_data::LandmarkGenerationData) :: Bool
    for pre::Term in operator.preconds
        for fact::FactPair in landmark.facts
            if FactPair(generation_data.term_index[pre], 1) == fact
                return true
            end
        end
    end
    return false
end

function is_causal_landmark(landmark::Landmark, generation_data::LandmarkGenerationData, state::Vector{FactPair}, spec::Specification) :: Bool
    if landmark.is_true_in_goal
        return true
    end

    excluded_op_ids::Vector{Int} = []
    excluded_props::Vector{FactPair} = []
    action_to_index::Dict{GroundAction, Int} = Dict(map(reverse, enumerate(generation_data.planning_graph.actions)))
    for operator::GroundAction in generation_data.planning_graph.actions
        if is_landmark_precondition(operator, landmark, generation_data)
            push!(excluded_op_ids, action_to_index[operator])
        end
    end

    reached::Dict{Pair{Int, Int}, Bool} = compute_relaxed_reachability(generation_data, state, excluded_props, excluded_op_ids)

    for fact::Term in spec.terms
        if !reached[Pair(generation_data.term_index[fact], 1)]
            return false
        end
    end
    return false
end

function discard_noncausal_landmarks(landmark_graph::LandmarkGraph, generation_data::LandmarkGenerationData, state::Vector{FactPair}, spec::Specification)
    landmark_graph_remove_node_if(landmark_graph, node::LandmarkNode -> !is_causal_landmark(node.landmark, generation_data, state, spec))
end

function compute_landmark_graph(domain::Domain, state::State, spec::Specification)# :: LandmarkGraph
    landmark_graph::LandmarkGraph = LandmarkGraph(0, 0, Dict(), Dict(), [])
    planning_graph::PlanningGraph = build_planning_graph(domain, state, spec)

    term_index = Dict(map(reverse, enumerate(planning_graph.conditions)))

    # Add goal landmarks
    for fact::Term in spec.terms
        fact_pair::Vector{FactPair} = [FactPair(term_index[fact], 1)]
        landmark_graph_add_landmark(landmark_graph, Landmark(fact_pair, false, false, true, false, Set(), Set()))
    end

    initial_state::Vector{FactPair} = map(s -> FactPair(term_index[s], 1), keys(state))
    initial_index = Dict(map(f -> Pair(f.var, f), initial_state))

    generation_data::LandmarkGenerationData = LandmarkGenerationData(planning_graph, term_index, Queue{Proposition}(), Set(), Dict(), Dict())
    for var in planning_graph.conditions
        for value in [0,1]
            fact_pair = FactPair(term_index[var], value)
            if !haskey(landmark_graph.simple_landmarks_to_nodes, fact_pair)
                landmark = Landmark([fact_pair], false, false, false, false, Set(), Set())
                if (haskey(initial_index, fact_pair.var) && initial_index[fact_pair.var].value == fact_pair.value) || !relaxed_task_solvable(landmark, initial_state, generation_data, spec)
                    landmark_graph_add_landmark(landmark_graph, landmark)
                end
            end
        end
    end

    discard_noncausal_landmarks(landmark_graph, generation_data, initial_state, spec)

    return Pair(landmark_graph, planning_graph)
end
