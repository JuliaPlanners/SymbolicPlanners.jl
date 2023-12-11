import Base.==
import Base.<
import DataStructures.Queue
import SymbolicPlanners.build_planning_graph

export compute_landmark_graph

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

function landmark_graph_remove_occurences(landmark_graph::LandmarkGraph, node::LandmarkNode) :: Nothing
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

function landmark_graph_remove_node(landmark_graph::LandmarkGraph, node::LandmarkNode) :: Nothing
    filter!(n -> n != node, landmark_graph.nodes)
end

function landmark_graph_remove_node_if(landmark_graph::LandmarkGraph, condition::Function) :: Nothing
    filter!(node -> begin
        if condition(node)
            return false
        end
        landmark_graph_remove_occurences(landmark_graph, node)
        return true
    end, landmark_graph.nodes)
end

function landmark_graph_set_landmark_ids(landmark_graph::LandmarkGraph) :: Nothing
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

function setup_exploration_queue(generation_data::LandmarkGenerationData, state::Vector{FactPair}, excluded_props::Vector{FactPair}, 
                                excluded_op_ids::Vector{Int})
    generation_data.propositions = Dict(map(f -> (f, Proposition(f, false, false)), [FactPair(x, y) for x in values(generation_data.planning_graph.conditions), y in [0, 1]]))

    for fact::FactPair in excluded_props
        propositions[fact].excluded = true
    end

    for fact::FactPair in state
        enqueue_if_necessary(generation_data.prop_queue, propositions[fact])
    end

    operation_index::Dict{GroundAction, Int} = map(reverse, enumerate(generation_data.planning_graph.actions))
    op_ids_to_mark::Set{Int} = Set(excluded_op_ids)
    for action::GroundAction in generation_data.planning_graph.actions
        for effect::Term in action.effect.add
            if isempty(action.preconds) && propositions[FactPair(generation_data.term_index[effect], 1)]
                push!(op_ids_to_mark, operation_index[action])
                break
            end
        end
    end

    generation_data.unsatisfied_preconditions = Dict()
    generation_data.excluded_operators = Set()
    for unary::GroundAction in filter(a -> length(a.term.args) == 1, generation_data.planning_graph.actions)
        generation_data.unsatisfied_preconditions[unary] = length(unary.preconds)

        if all(map(propositions[e -> FactPair(term_index[e], 1)].excluded, unary.effect.add)) || operation_index[unary] in op_ids_to_mark
            push!{generation_data.excluded_operators, unary}
        else
            delete!(generation_data.excluded_operators, unary)

            if generation_data.unsatisfied_preconditions[unary] == 0
                enqueue_if_necessary(generation_data.prop_queue, propositions[fact])
            end
        end
    end

    for fact::FactPair in excluded_props
        propositions[fact].excluded = false
    end
end

function relaxed_exploration(generation_data::LandmarkGenerationData)
    while !isempty(generation_data.prop_queue)
        prop::Proposition = dequeue!(generation_data.prop_queue)

        triggered_operators::Vector{GroundAction} = map(i -> generation_data.planning_graph.actions[i], generation_data.planning_graph.effect_map[term_index[prop.fact]])
        for operator::GroundAction in triggered_operators
            if !(operator in generation_data.excluded_operators)
                generation_data.unsatisfied_preconditions[operator] -= 1
                
                if generation_data.unsatisfied_preconditions[operator] == 0
                    for effect::Term in operator.effect.add
                        enqueue_if_necessary(generation_data.prop_queue, effect)
                    end
                end
            end
        end
    end
end

function compute_relaxed_reachability(state::State, excluded_props::Vector{FactPair}, excluded_op_ids::Vector{Int}) :: Dict{Pair{Int, Int}, Bool}
    setup_exploration_queue(generation_data, state, excluded_props, excluded_op_ids)
    relaxed_exploration(generation_data)

    reached::Dict{Pair{Int, Int}, Bool} = Dict()
    for prop::Term in generation_data.propositions
        if prop.reached
            reached[Pair(prop.var, prop.value)] = true
        end
    end
    return reached
end

function relaxed_task_solvable(landmark::Landmark, state::State, generation_data::LandmarkGenerationData) :: Bool
    excluded_op_ids::Vector{Int} = []
    excluded_props::Vector{FactPair} = landmark.facts

    reached::Dict{Pair{Int, Int}, Bool} = compute_relaxed_reachability(state, excluded_props, excluded_op_ids)
    for fact::Term in spec.facts
        if !reached[Pair(generation_data.term_index[fact], 1)]
            return false
        end
    end

    return true
end

function compute_landmark_graph(domain::Domain, state::State, spec::Specification)# :: LandmarkGraph
    landmark_graph = LandmarkGraph(0, 0, Dict(), Dict(), [])

    planning_graph = build_planning_graph(domain, state, spec)

    term_index = Dict(map(reverse, enumerate(planning_graph.conditions)))

    for fact::Term in spec.terms
        fact_pair::Vector{FactPair} = [FactPair(term_index[fact], 1)]
        landmark_graph_add_landmark(landmark_graph, Landmark(fact_pair, false, false, true, false, Set(), Set()))
    end

    initial_state::Vector{FactPair} = map(s -> FactPair(term_index[s], 1), keys(state))
    initial_index = Dict(map(reverse, enumerate(initial_state)))

    generation_data::LandmarkGenerationData = LandmarkGenerationData(planning_graph, term_index, Queue{Proposition}(), Set(), Dict(), Dict())
    for var in planning_graph.conditions
        for value in [0,1]
            fact_pair = FactPair(term_index[var], value)
            if haskey(landmark_graph.simple_landmarks_to_nodes, fact_pair)
                landmark = Landmark([fact_pair], false, false, false, false, Set(), Set())
                if get(initial_index, fact_pair, FactPair(-1, 0)).value == fact_pair.value || !relaxed_task_solvable(landmark, state, generation_data)
                    landmark_graph_add_landmark(landmark_graph, landmark)
                end
            end
        end
    end

    return landmark_graph
end
