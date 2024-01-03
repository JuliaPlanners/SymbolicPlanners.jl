import Base.==
import Base.<
import Base.hash
import Base.isless
import DataStructures.Queue
import SymbolicPlanners.build_planning_graph

export compute_landmark_graph
export compute_relaxed_landmark_graph
export approximate_reasonable_orders

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
function isless(l::FactPair, r::FactPair)
    return l < r
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


mutable struct Proposition
    fact::FactPair
    precondition_of::Vector{Int}
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

mutable struct UnaryOperator
    action::GroundAction
    num_preconditions::Int
    effect::Proposition
    unsatisfied_preconditions::Int
    excluded::Bool
end

mutable struct LandmarkGenerationData
    planning_graph::PlanningGraph
    term_index::Dict{Term, Int}
    prop_queue::Queue{Proposition}
    excluded_operators::Set{GroundAction}
    unsatisfied_preconditions::Dict{GroundAction, Int}
    propositions::Dict{FactPair, Proposition}
    unary_operators::Vector{UnaryOperator}
    initial_state::Vector{FactPair}
end

function build_unary_operators(generation_data::LandmarkGenerationData)
    for action::GroundAction in generation_data.planning_graph.actions
        precondition_facts1::Vector{FactPair} = []

        append!(precondition_facts1, map(t -> FactPair(generation_data.term_index[t], 1), action.preconds))
        sort!(precondition_facts1)

        for effect::Term in action.effect.add
            precondition::Vector{Proposition} = []
            push!(precondition, map(f -> generation_data.propositions[f], precondition_facts1)...)

            effect_proposition::Proposition = generation_data.propositions[FactPair(generation_data.term_index[effect], 1)]
            push!(generation_data.unary_operators, UnaryOperator(action, length(precondition), effect_proposition, -1, false))

            for pre::Proposition in precondition
                push!(pre.precondition_of, length(generation_data.unary_operators))
            end
        end
        for effect::Term in action.effect.del
            precondition::Vector{Proposition} = []
            push!(precondition, map(f -> generation_data.propositions[f], precondition_facts1)...)

            effect_proposition::Proposition = generation_data.propositions[FactPair(generation_data.term_index[effect], 0)]
            push!(generation_data.unary_operators, UnaryOperator(action, length(precondition), effect_proposition, -1, false))

            for pre::Proposition in precondition
                push!(pre.precondition_of, length(generation_data.unary_operators))
            end
        end
    end
end

function setup_exploration_queue(generation_data::LandmarkGenerationData, state::Vector{FactPair}, excluded_props::Vector{FactPair}, excluded_op_ids::Vector{Int})
    empty!(generation_data.prop_queue)

    if length(generation_data.propositions) == 0
        generation_data.propositions = Dict(map(f -> (f, Proposition(f, [], false, false)), [FactPair(generation_data.term_index[x], y) for x in values(generation_data.planning_graph.conditions), y in [0, 1]]))
    else
        for prop::Proposition in values(generation_data.propositions)
            prop.reached = false
        end
    end
    if length(generation_data.unary_operators) == 0
        build_unary_operators(generation_data)
    end

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
            if isempty(action.preconds) && generation_data.propositions[FactPair(generation_data.term_index[effect], 1)].excluded
                push!(op_ids_to_mark, operation_index[action])
                break
            end
        end
    end

    generation_data.unsatisfied_preconditions = Dict()
    for unary::UnaryOperator in generation_data.unary_operators
        unary.unsatisfied_preconditions = unary.num_preconditions

        if unary.effect.excluded || operation_index[unary.action] in op_ids_to_mark
            unary.excluded = true
        else
            unary.excluded = false

            if unary.unsatisfied_preconditions == 0
                enqueue_if_necessary(generation_data.prop_queue, unary.effect)
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

        triggered_operators::Vector{UnaryOperator} = map(i -> generation_data.unary_operators[i], prop.precondition_of)
        for operator::UnaryOperator in triggered_operators
            if !operator.excluded
                operator.unsatisfied_preconditions -= 1
                
                if operator.unsatisfied_preconditions == 0
                    enqueue_if_necessary(generation_data.prop_queue, operator.effect)
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
            return true
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
    initial_index::Dict{Int, FactPair} = Dict(map(f -> Pair(f.var, f), initial_state))

    generation_data::LandmarkGenerationData = LandmarkGenerationData(planning_graph, term_index, Queue{Proposition}(), Set(), Dict(), Dict(), [], initial_state)
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

    return Pair(landmark_graph, generation_data)
end

function compute_relaxed_landmark_graph(domain::Domain, state::State, spec::Specification)# :: LandmarkGraph
    landmark_graph::LandmarkGraph = LandmarkGraph(0, 0, Dict(), Dict(), [])
    planning_graph::PlanningGraph = build_planning_graph(domain, state, spec)

    term_index = Dict(map(reverse, enumerate(planning_graph.conditions)))

    initial_state::Vector{FactPair} = map(s -> FactPair(term_index[s], 1), keys(state))
    generation_data::LandmarkGenerationData = LandmarkGenerationData(planning_graph, term_index, Queue{Proposition}(), Set(), Dict(), Dict(), [], initial_state)

    dtg_successors::Dict{Int, Dict{Int, Set{Int}}} = build_dtg_successors(generation_data)
    disjunction_classes::Dict{Int, Dict{Int, Int}} = build_disjunction_classes(generation_data)

    open_landmarks::Queue{LandmarkNode} = Queue{LandmarkNode}()
    for fact::Term in spec.terms
        fact_pair::Vector{FactPair} = [FactPair(term_index[fact], 1)]
        landmark = Landmark(fact_pair, false, false, true, false, Set(), Set())
        lm_node = landmark_graph_add_landmark(landmark_graph, landmark)
        enqueue!(open_landmarks, lm_node)
    end

    forward_orders::Dict{LandmarkNode, Vector{FactPair}} = Dict()
    while !isempty(open_landmarks)
        lm_node::LandmarkNode = dequeue!(open_landmarks)
        landmark::Landmark = lm_node.landmark

        if !landmark_is_true_in_state(landmark, initial_state)
            excluded_op_ids::Vector{Int} = []
            excluded_props::Vector{FactPair} = landmark.facts
            reached::Dict{Pair{Int, Int}, Bool} = compute_relaxed_reachability(generation_data, initial_state, excluded_props, excluded_op_ids)

            shared_pre::Dict{Int, Int} = compute_shared_preconditions(generation_data, reached, landmark)
            for pre::Pair{Int, Int} in shared_pre
                found_simple_lm_and_order(landmark_graph, FactPair(pre.first, pre.second), lm_node, GREEDY_NECESSARY, open_landmarks, forward_orders)
            end
            approximate_lookahead_orders(generation_data, reached, lm_node, dtg_successors, landmark_graph, open_landmarks, forward_orders)

            disjunctive_pre::Vector{Set{FactPair}} = compute_disjunctive_preconditions(generation_data, reached, landmark, disjunction_classes, landmark_graph)
            for preconditions::Set{FactPair} in disjunctive_pre
                if length(preconditions) < 5
                    found_disj_lm_and_order(generation_data, preconditions, lm_node, GREEDY_NECESSARY, landmark_graph, open_landmarks)
                end
            end
        end
    end

    add_lm_forward_orders(landmark_graph, forward_orders)

    # if (!disjunctive_landmarks) {
    #     discard_disjunctive_landmarks();
    # }

    # if (!use_orders) {
    #     discard_all_orderings();
    # }

    # if (only_causal_landmarks) {
    #     discard_noncausal_landmarks(task_proxy, exploration);
    # }

    return Pair(landmark_graph, generation_data)
end

function add_lm_forward_orders(landmark_graph::LandmarkGraph, forward_orders::Dict{LandmarkNode, Vector{FactPair}})
    for node::LandmarkNode in landmark_graph.nodes
        for node2_pair::FactPair in get(forward_orders, node, [])
            if haskey(landmark_graph.simple_landmarks_to_nodes, node2_pair)
                node2::LandmarkNode = landmark_graph.simple_landmarks_to_nodes[node2_pair]
                edge_add(node, node2, NATURAL)
            end
        end
        if haskey(forward_orders, node)
            empty!(forward_orders[node])
        end
    end
end

function found_disj_lm_and_order(generation_data::LandmarkGenerationData, a::Set{FactPair}, b::LandmarkNode, t::EdgeType, landmark_graph::LandmarkGraph, open_landmarks::Queue{LandmarkNode})
    simple_lm_exists::Bool = false
    lm_prop::FactPair = no_fact()
    initial_state::Dict{Int, Int} = Dict()
    for fp::FactPair in generation_data.initial_state
        initial_state[fp.var] = fp.value
    end

    for lm::FactPair in a
        if get(initial_state, lm.var, 0) == lm.value
            return
        end
        if haskey(landmark_graph.simple_landmarks_to_nodes, lm)
            simple_lm_exists = true
            lm_prop = lm
            break
        end
    end

    new_lm_node::LandmarkNode
    if simple_lm_exists
        return
    elseif landmark_graph_overlaps_disjunctive(landmark_graph, a)
        if landmark_graph_contains_disjunctive(landmark_graph, a)
            new_lm_node = landmark_graph.disjunctive_landmarks_to_nodes[first(a)]
            edge_add(new_lm_node, b, t)
            return
        end
        return
    end

    landmark::Landmark = Landmark(collect(a), true, false, false, false, Set(), Set())
    new_lm_node = landmark_graph_add_landmark(landmark_graph, landmark)
    enqueue!(open_landmarks, new_lm_node)
    edge_add(new_lm_node, b, t)
end

function compute_disjunctive_preconditions(generation_data::LandmarkGenerationData, reached, landmark::Landmark, disjunction_classes::Dict{Int, Dict{Int, Int}}, landmark_graph::LandmarkGraph)
    op_ids::Vector{Int} = []
    for lm_fact::FactPair in landmark.facts
        tmp_op_ids::Vector{Int} = get_operators_including_eff(generation_data, lm_fact)
        append!(op_ids, tmp_op_ids)
    end

    num_ops::Int = 0
    preconditions::Dict{Int, Vector{FactPair}} = Dict()
    used_operators::Dict{Int, Set{Int}} = Dict()
    for i::Int in range(1, length(op_ids))
        op::Int = op_ids[i]
        if possibly_reaches_lm(generation_data, op, reached, landmark)
            num_ops += 1
            next_pre::Dict{Int, Int} = get_greedy_preconditions_for_lm(generation_data, landmark, op)
            for pre::Pair{Int, Int} in pairs(next_pre)
                disj_class::Int = disjunction_classes[pre.first][pre.second]
                if disj_class == -1
                    continue
                end

                pre_fact::FactPair = FactPair(pre.first, pre.second)
                if !haskey(landmark_graph.simple_landmarks_to_nodes, pre_fact)
                    if !haskey(preconditions, disj_class)
                        preconditions[disj_class] = []
                        used_operators[disj_class] = Set()
                    end

                    push!(preconditions[disj_class], pre_fact)
                    push!(used_operators[disj_class], i)
                end
            end
        end
    end

    disjunctive_pre::Vector{Set{FactPair}} = []
    for pre::Pair{Int, Vector{FactPair}} in preconditions
        if length(used_operators[pre.first]) == num_ops
            pre_set::Set{FactPair} = Set(pre.second)
            if length(pre_set) > 1
                push!(disjunctive_pre, pre_set)
            end
        end
    end

    return disjunctive_pre
end

function approximate_lookahead_orders(generation_data::LandmarkGenerationData, reached::Dict{Pair{Int, Int}, Bool}, lmp::LandmarkNode, dtg_successors::Dict{Int, Dict{Int, Set{Int}}}, landmark_graph::LandmarkGraph, open_landmarks::Queue{LandmarkNode}, forward_orders::Dict{LandmarkNode, Vector{FactPair}})
    find_forward_orders(generation_data, reached, lmp, forward_orders)

    landmark::Landmark = lmp.landmark
    if (landmark.disjunctive)
        return
    end
    lm_fact::FactPair = landmark.facts[1]

    unreached::Set{Int} = Set()
    for value::Int in [0,1]
        if !reached[Pair(lm_fact.var, value)] && lm_fact.value != value
            push!(unreached, value)
        end
    end

    for value::Int in [0,1]
        if !(value in unreached) && lm_fact.value != value
            exclude::Set{Int} = unreached
            push!(exclude, value)
            
            if !domain_connectivity(generation_data, lm_fact, exclude, dtg_successors)
                found_simple_lm_and_order(landmark_graph, FactPair(lm_fact.var, value), lmp, NATURAL, open_landmarks, forward_orders)
            end
        end
    end
end

function domain_connectivity(generation_data::LandmarkGenerationData, landmark::FactPair, exclude::Set{Int}, dtg_successors::Dict{Int, Dict{Int, Set{Int}}}) :: Bool
    initial_state::Dict{Int, Int} = Dict()
    for fp::FactPair in generation_data.initial_state
        initial_state[fp.var] = fp.value
    end
    var::Int = landmark.var
    value::Int = get(initial_state, var, 0)
    if value in exclude
        return false
    end

    open::Queue{Int} = Queue{Int}()
    closed::Set{Int} = exclude
    enqueue!(open, value)
    push!(closed, value)
    successors::Dict{Int, Set{Int}} = dtg_successors[var]

    while !(landmark.value in closed)
        if isempty(open)
            return false
        end
        c::Int = dequeue!(open)
        for val::Int in successors[c]
            if !(val in closed)
                enqueue!(open, val)
                push!(closed, val)
            end
        end
    end
    return true
end

function find_forward_orders(generation_data::LandmarkGenerationData, reached::Dict{Pair{Int, Int}, Bool}, lm_node::LandmarkNode, forward_orders::Dict{LandmarkNode, Vector{FactPair}})
    for var::Term in generation_data.planning_graph.conditions
        for value::Int in [0,1]
            fact_id::Int = generation_data.term_index[var]
            if reached[Pair(fact_id, value)]
                continue
            end
            fact::FactPair = FactPair(fact_id, value)

            insert::Bool = true
            for lm_fact::FactPair in lm_node.landmark.facts
                if fact != lm_fact
                    intersection_empty::Bool = true

                    effect::Term = generation_data.planning_graph.conditions[fact.var]
                    if fact.value == 0
                        effect = Compound(:not, [effect])
                    end
                    reach_fact::Vector{Int} = generation_data.planning_graph.effect_map[effect]
        
                    effect_lm::Term = generation_data.planning_graph.conditions[lm_fact.var]
                    if lm_fact.value == 0
                        effect_lm = Compound(:not, [effect_lm])
                    end
                    reach_lm::Vector{Int} = generation_data.planning_graph.effect_map[effect_lm]

                    for j::Int in range(1, length(reach_fact))
                        if !intersection_empty
                            break
                        end
                        for k::Int in range(1, length(reach_lm))
                            if !intersection_empty
                                break
                            end
                            
                            if reach_fact[j] == reach_lm[k]
                                intersection_empty = false
                            end
                        end
                    end

                    if !intersection_empty
                        insert = false
                        break
                    end
                else
                    insert = false
                    break
                end
            end

            if insert
                if !haskey(forward_orders, lm_node)
                    forward_orders[lm_node] = []
                end
                push!(forward_orders[lm_node], fact)
            end
        end
    end
end

function DummyLandmarkNode()
    return LandmarkNode(-1, Landmark([], false, false, false, false, Set(), Set()), Dict(), Dict())
end

function found_simple_lm_and_order(landmark_graph::LandmarkGraph, a::FactPair, b::LandmarkNode, t::EdgeType, open_landmarks::Queue{LandmarkNode}, forward_orders::Dict{LandmarkNode, Vector{FactPair}})
    simple_lm::LandmarkNode = DummyLandmarkNode()
    if haskey(landmark_graph.simple_landmarks_to_nodes, a)
        simple_lm = landmark_graph.simple_landmarks_to_nodes[a]
        edge_add(simple_lm, b, t)
        return
    end

    landmark::Landmark = Landmark([a], false, false, false, false, Set(), Set())
    if haskey(landmark_graph.disjunctive_landmarks_to_nodes, a)
        disj_lm::LandmarkNode = landmark_graph.disjunctive_landmarks_to_nodes[a]
        for lm in findall(l -> l == disj_lm, open_landmarks)
            delete!(open_landmarks, lm)
        end
        delete!(forward_orders, disj_lm)

        predecessors::Vector{LandmarkNode} = []
        for pred in keys(disj_lm.parents)
            push!(predecessors, pred)
        end

        landmark_graph_remove_node(landmark_graph, disj_lm)

        simple_lm = landmark_graph_add_landmark(landmark_graph, landmark)
        enqueue!(open_landmarks, simple_lm)
        edge_add(simple_lm, b, t)

        for pred::LandmarkNode in predecessors
            edge_add(pred, simple_lm, NATURAL)
        end
    else
        simple_lm = landmark_graph_add_landmark(landmark_graph, landmark)
        enqueue!(open_landmarks, simple_lm)
        edge_add(simple_lm, b, t)
    end
end

function build_dtg_successors(generation_data::LandmarkGenerationData) :: Dict{Int, Dict{Int, Set{Int}}}
    dtg_successors::Dict{Int, Dict{Int, Set{Int}}} = Dict()
    for op::GroundAction in generation_data.planning_graph.actions
        precondition_map::Dict{Int, Int} = Dict()
        for precondition::Term in op.preconds
            precondition_map[generation_data.term_index[precondition]] = 1
        end

        effects::Vector{FactPair} = map(t -> FactPair(generation_data.term_index[t], 1), op.effect.add)
        append!(effects, map(t -> FactPair(generation_data.term_index[t], 0), op.effect.del))
        for effect_fact::FactPair in effects
            var_id::Int = effect_fact.var
            post::Int = effect_fact.value
            if haskey(precondition_map, var_id)
                pre::Int = precondition_map[var_id]
                add_dtg_successor(dtg_successors, var_id, pre, post)
            else
                add_dtg_successor(dtg_successors, var_id, 0, post)
                add_dtg_successor(dtg_successors, var_id, 1, post)
            end
        end
    end
    return dtg_successors
end

function build_disjunction_classes(generation_data::LandmarkGenerationData) :: Dict{Int, Dict{Int, Int}}
    predicate_to_index::Dict{Symbol, Int} = Dict()
    disjunction_classes::Dict{Int, Dict{Int, Int}} = Dict()
    for var::Term in generation_data.planning_graph.conditions
        var_id::Int = generation_data.term_index[var]
        for value::Int in [0,1]
            predicate::Symbol = var.name

            disj_class::Int = -1
            if predicate in keys(predicate_to_index)
                disj_class = predicate_to_index[predicate]
            else
                disj_class = length(predicate_to_index)
                predicate_to_index[predicate] = disj_class
            end

            if !haskey(disjunction_classes, var_id)
                disjunction_classes[var_id] = Dict()
            end
            disjunction_classes[var_id][value] = disj_class
        end
    end
    return disjunction_classes
end

function compute_shared_preconditions(generation_data::LandmarkGenerationData, reached::Dict{Pair{Int, Int}, Bool}, landmark::Landmark)
    init::Bool = true
    shared_pre::Dict{Int, Int} = Dict()
    for lm_fact::FactPair in landmark.facts
        op_ids = get_operators_including_eff(generation_data, lm_fact)

        for op_id in op_ids
            if !init && isempty(shared_pre)
                break
            end

            if possibly_reaches_lm(generation_data, op_id, reached, landmark)
                next_pre::Dict{Int, Int} = get_greedy_preconditions_for_lm(generation_data, landmark, op_id)
                if init
                    init = false
                    shared_pre = next_pre
                else
                    _intersect(shared_pre, next_pre)
                end
            end
        end
    end
    return shared_pre
end

function _intersect(a::Dict{Int, Int}, b::Dict{Int, Int}) :: Dict{Int, Int}
    if length(a) > length(b)
        return _intersect(b, a)
    end

    result::Dict{Int, Int} = Dict()
    for pair_a::Pair in pairs(a)
        if haskey(b, pair_a.first) && b[pair_a.first] == pair_a.second
            result[pair_a.first] = pair_a.second
        end
    end
    return result
end

function get_greedy_preconditions_for_lm(generation_data::LandmarkGenerationData, landmark::Landmark, op_id::Int) :: Dict{Int, Int}
    result::Dict{Int, Int} = Dict()
    has_precondition_on_var::Vector{Bool} = fill(false, length(generation_data.planning_graph.conditions))
    for precondition::Int in reduce(vcat, generation_data.planning_graph.act_parents[op_id])
        result[precondition] = 1
        has_precondition_on_var[precondition] = true;
    end

    initial_state::Dict{Int, Int} = Dict(map(fp -> Pair(fp.var, fp.value), generation_data.initial_state))
    for effect::Int in generation_data.planning_graph.act_children[op_id]
        if !has_precondition_on_var[effect]
            for lm_fact::FactPair in landmark.facts
                value::Int = get(initial_state, effect, 0)
                if lm_fact.var == effect && value != lm_fact.value
                    result[effect] = value
                    break
                end
            end
        end
    end

    # lm_props_achievable::Set{Int} = Set()
    # for effect::Int in generation_data.planning_graph.act_children[op_id]
    #     effect_fact::FactPair = FactPair(effect, 1)
    #     for j in range(1, length(landmark.facts))
    #         if landmark.facts[j] == effect_fact
    #             push!(lm_props_achievable, j)
    #         end
    #     end
    # end

    # intersection::Dict{Int, Int} = Dict()
    # init::Bool = true
    # for lm_prop::Int in lm_props_achievable
    #     for effect::Int in generation_data.planning_graph.act_children[op_id]
    #         if !init && isempty(intersection)
    #             break
    #         end

    #         effect_fact::FactPair = FactPair(effect, 1)
    #         current_cond::Dict{Int, Int} = Dict()
    #         if landmark.facts[lm_prop] == effect_fact

    #         end
    #     end
    # end
    return result
end

function possibly_reaches_lm(generation_data::LandmarkGenerationData, op_id::Int, reached::Dict{Pair{Int, Int}, Bool}, landmark::Landmark) :: Bool
    preconditions::Vector{Int} = reduce(vcat, generation_data.planning_graph.act_parents[op_id])
    for pre in preconditions
        if !reached[Pair(pre, 1)]
            return false
        end
    end

    for effect in generation_data.planning_graph.act_children[op_id]
        for fact::FactPair in landmark.facts
            if FactPair(effect, 1) == fact
                return true
            end
        end
    end

    return false
end

function get_operators_including_eff(generation_data::LandmarkGenerationData, factpair::FactPair) :: Vector{Int}
    effect::Term = generation_data.planning_graph.conditions[factpair.var]
    if factpair.value == 0
        effect = Compound(:not, [effect])
    end
    return get(generation_data.planning_graph.effect_map, effect, [])
end

function add_dtg_successor(dtg_successors::Dict{Int, Dict{Int, Set{Int}}}, var::Int, pre::Int, post::Int)
    if pre != post
        if !haskey(dtg_successors, var)
            dtg_successors[var] = Dict()
        end
        if !haskey(dtg_successors[var], pre)
            dtg_successors[var][pre] = Set()
        end
        push!(dtg_successors[var][pre], post)
    end
end

function edge_add(from::LandmarkNode, to::LandmarkNode, type::EdgeType)
    if !haskey(from.children, to) || from.children[to] >= type
        from.children[to] = type
        to.parents[from] = type
    end
end

function approximate_reasonable_orders(landmark_graph::LandmarkGraph, generation_data::LandmarkGenerationData)
    for node::LandmarkNode in landmark_graph.nodes
        landmark::Landmark = node.landmark
        if landmark.disjunctive
            continue
        end

        if landmark.is_true_in_goal
            for node2::LandmarkNode in landmark_graph.nodes
                landmark2::Landmark = node2.landmark
                if (landmark == landmark2 || landmark2.disjunctive)
                    continue
                end
                if interferes(landmark2, landmark, generation_data)
                    edge_add(node2, node, REASONABLE)
                end
            end
        else
            interesting_nodes::Set{LandmarkNode} = Set()
            for child::Pair{LandmarkNode, EdgeType} in node.children
                node2::LandmarkNode = child.first
                edge2::EdgeType = child.second
                if edge2 >= GREEDY_NECESSARY
                    for parent::Pair{LandmarkNode, EdgeType} in node2.parents
                        parent_node::LandmarkNode = parent.first
                        edge::EdgeType = parent.second
                        if parent_node.landmark.disjunctive
                            continue
                        end
                        if edge >= NATURAL && parent_node != node
                            push!(interesting_nodes, parent_node)
                            collect_ancestors(interesting_nodes, parent_node);
                        end
                    end
                end
            end

            for node2::LandmarkNode in interesting_nodes
                landmark2::Landmark = node2.landmark
                if landmark == landmark2 || landmark2.disjunctive
                    continue
                end
                if interferes(landmark2, landmark, generation_data)
                    edge_add(node2, node, REASONABLE)
                end
            end
        end
    end
end

function is_mutex(fact_a::FactPair, fact_b::FactPair)
    return fact_a.var == fact_b.var && fact_a.value != fact_b.value
end

function interferes(landmark_a::Landmark, landmark_b::Landmark, generation_data::LandmarkGenerationData) :: Bool
    for lm_fact_b::FactPair in landmark_b.facts
        for lm_fact_a::FactPair in landmark_a.facts
            if lm_fact_a == lm_fact_b
                if !landmark_a.conjunctive || !landmark_b.conjunctive
                    return false
                else
                    continue
                end
            end

            if landmark_a.conjunctive
                continue
            end

            if is_mutex(lm_fact_a, lm_fact_b)
                return true
            end

            shared_eff::Dict{Int, Int} = Dict()
            init::Bool = true
            effect::Term = generation_data.planning_graph.conditions[lm_fact_a.var]
            if lm_fact_a.value == 0
                effect = Compound(:not, [effect])
            end
            op_ids::Vector{Int} = generation_data.planning_graph.effect_map[effect]
            for op_id::Int in op_ids
                if !init && isempty(shared_eff)
                    break
                end

                op::GroundAction = generation_data.planning_graph.actions[op_id]
                effects::Vector{FactPair} = map(t -> FactPair(generation_data.term_index[t], 1), op.effect.add)
                append!(effects, map(t -> FactPair(generation_data.term_index[t], 0), op.effect.del))

                next_eff::Dict{Int, Int} = Dict()
                for effect_fact::FactPair in effects
                    if effect_fact.var != lm_fact_a
                        next_eff[effect_fact.var] = effect_fact.value
                    end
                end

                if init
                    shared_eff = next_eff
                else
                    result::Dict{Int, Int} = Dict()
                    for eff1::Pair{Int, Int} in shared_eff
                        if haskey(next_eff, eff1.first) && next_eff[eff1.first] == eff1.second
                            result[eff1.first] = eff1.second
                        end
                    end
                    shared_eff = result
                end
                init = false
            end

            for eff::Pair{Int, Int} in shared_eff
                effect_fact::FactPair = FactPair(eff.first, eff.second)
                if effect_fact != lm_fact_a && effect_fact != lm_fact_b && is_mutex(effect_fact, lm_fact_b)
                    return true
                end
            end
        end
    end

    return false
end

function collect_ancestors(result::Set{LandmarkNode}, node::LandmarkNode)
    open_nodes::Queue{LandmarkNode} = Queue{LandmarkNode}()
    closed_nodes::Set{LandmarkNode} = Set()
    for p::Pair{LandmarkNode, EdgeType} in node.parents
        parent = p.first
        edge = p.second
        if edge >= NATURAL && !(parent in closed_nodes)
            enqueue!(open_nodes, parent)
            push!(closed_nodes, parent)
            push!(result, parent)
        end
    end
    while !isempty(open_nodes)
        node2::LandmarkNode = first(open_nodes)
        for p::Pair{LandmarkNode, EdgeType} in node2.parents
            parent = p.first
            edge = p.second
            if edge >= NATURAL && !(parent in closed_nodes)
                enqueue!(open_nodes, parent)
                push!(closed_nodes, parent)
                push!(result, parent)
            end
        end
        dequeue!(open_nodes)
    end
end
