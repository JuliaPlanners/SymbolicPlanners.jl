import Base.==
import Base.<

import SymbolicPlanners: build_planning_graph

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
    end
end

function compute_landmark_graph(domain::Domain, state::State, spec::Specification)# :: LandmarkGraph
    landmark_graph = LandmarkGraph(0, 0, Dict(), Dict(), [])

    planning_graph = build_planning_graph(domain, state, spec)

    for fact in spec.terms
        var::Int = findall(x -> x == fact, planning_graph.conditions)[1]
        fact_pair::Vector{FactPair} = [FactPair(var, -1)]
        landmark_graph_add_landmark(landmark_graph, Landmark(fact_pair, false, false, true, false, Set(), Set()))
    end

    #temp return
    return planning_graph
end

# generate_relaxed_landmarks(const shared_ptr<AbstractTask> &task, Exploration &exploration) {
#     TaskProxy task_proxy(*task);
#     if (log.is_at_least_normal()) {
#         log << "Generating landmarks by testing all facts with RPG method" << endl;
#     }

#     // insert goal landmarks and mark them as goals
#     for (FactProxy goal : task_proxy.get_goals()) {
#         Landmark landmark({goal.get_pair()}, false, false, true);
#         lm_graph->add_landmark(move(landmark));
#     }
#     // test all other possible facts
#     State initial_state = task_proxy.get_initial_state();
#     for (VariableProxy var : task_proxy.get_variables()) {
#         for (int value = 0; value < var.get_domain_size(); ++value) {
#             const FactPair lm(var.get_id(), value);
#             if (!lm_graph->contains_simple_landmark(lm)) {
#                 Landmark landmark({lm}, false, false);
#                 if (initial_state[lm.var].get_value() == lm.value || !relaxed_task_solvable(task_proxy, exploration, landmark)) {
#                     lm_graph->add_landmark(move(landmark));
#                 }
#             }
#         }
#     }

#     if (only_causal_landmarks) {
#         discard_noncausal_landmarks(task_proxy, exploration);
#     }
# }

# mutable struct GenericState <: State
#     types::Set{Compound} # Object type declarations
#     facts::Set{Term} # Boolean-valued fluents
#     values::Dict{Symbol,Any} # All other fluents
# end

# @kwdef mutable struct GenericProblem <: Problem
#     name::Symbol # Name of problem
#     domain::Symbol = gensym() # Name of associated domain
#     objects::Vector{Const} = Const[] # List of objects
#     objtypes::Dict{Const,Symbol} = Dict{Const,Symbol}() # Types of objects
#     init::Vector{Term} = Term[] # Predicates that hold in initial state
#     goal::Term = Compound(:and, []) # Goal formula
#     metric::Union{Term,Nothing} = nothing # Metric formula
#     constraints::Union{Term,Nothing} = nothing # Constraints formula
# end
# @kwdef mutable struct GenericDomain <: Domain
#     name::Symbol # Name of domain
#     requirements::Dict{Symbol,Bool} = # PDDL requirements
#         copy(DEFAULT_REQUIREMENTS)
#     typetree::Dict{Symbol,Vector{Symbol}} = # Types and their subtypes
#         Dict(:object => Symbol[])
#     datatypes::Dict{Symbol,Type} = # Non-object data types
#         Dict{Symbol,Type}()
#     constants::Vector{Const} = # List of object constants
#         Const[]
#     constypes::Dict{Const,Symbol} = # Types of constants
#         Dict{Const,Symbol}()
#     predicates::Dict{Symbol,Signature} = # Predicate signatures
#         Dict{Symbol,Signature}()
#     functions::Dict{Symbol,Signature} = # Function signatures
#         Dict{Symbol,Signature}()
#     funcdefs::Dict{Symbol,Any} = # External function implementations
#         Dict{Symbol,Any}()
#     axioms::Dict{Symbol,Clause} = # Axioms / derived predicates
#         Dict{Symbol,Clause}()
#     actions::Dict{Symbol,Action} = # Action definitions
#         Dict{Symbol,Any}()
#     _extras::Dict{Symbol,Any} = # Extra fields for PDDL extensions
#         Dict{Symbol,Any}()
# end
