import Base.==
import Base.<
import Base.hash
import Base.isless

export FactPair, no_fact
export Landmark, landmark_is_true_in_state
export EdgeType
export LandmarkNode, node_from_landmark
export LandmarkGraph
export landmark_graph_get_num_edges, landmark_graph_get_node, landmark_graph_overlaps_disjunctive
export landmark_graph_contains_disjunctive, landmark_graph_contains_landmark, landmark_graph_add_landmark
export landmark_graph_remove_occurences, landmark_graph_remove_node, landmark_graph_remove_node_if
export landmark_graph_set_landmark_ids


mutable struct FactPair
    var::Int
    value::Int
end

function (<)(l::FactPair, r::FactPair)
    return l.var < r.var || (l.var == r.var && l.value < r.value)
end
function isless(l::FactPair, r::FactPair)
    return l < r
end
function (==)(l::FactPair, r::FactPair)
    return l.var == r.var && l.value == r.value
end
function hash(f::FactPair, h::UInt)
    return hash(f.var, hash(f.value, hash(FactPair, h)))
end

function no_fact() :: FactPair
    return FactPair(-1, -1)
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

function landmark_is_true_in_state(landmark::Landmark, p_graph::PlanningGraph, state::State) :: Bool
    if landmark.disjunctive
        for fact::FactPair in landmark.facts
            if state[p_graph.conditions[fact.var]] == fact.value
                return true
            end
        end
        return false
    else
        for fact::FactPair in landmark.facts
            if state[p_graph.conditions[fact.var]] != fact.value
                return false
            end
        end
        return true
    end
end

function landmark_is_true_in_state(landmark::Landmark, state::Vector{FactPair}) :: Bool
    state_dict::Dict{Int, Int} = Dict()
    for fp::FactPair in state
        state_dict[fp.var] = fp.value
    end
    if landmark.disjunctive
        for fact::FactPair in landmark.facts
            if get(state_dict, fact.var, 0) == fact.value
                return true
            end
        end
        return false
    else
        for fact::FactPair in landmark.facts
            if get(state_dict, fact.var, 0) != fact.value
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
