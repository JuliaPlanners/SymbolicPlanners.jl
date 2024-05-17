## The LM-Cut Heuristic ##
export LM_CutHeuristic
using DataStructures: Queue
"""
    LM_CutHeuristic()

A landmark based heuristic, which builds on top of the existing 
relaxed planning graph heuristic HMax. It functions by finding sets of actions
through which an optimal plan must pass (the landmarks), and then choosing the lowest cost one.
The heuristic is the sum of these minima across all the sets of landmarks.
"""
mutable struct LM_CutHeuristic <: Heuristic 
    graph::PlanningGraph
    statics::Vector{Symbol}
    LM_CutHeuristic() = new()
end

is_precomputed(h::LM_CutHeuristic) = isdefined(h, :graph)

function Base.show(io::IO, h::LM_CutHeuristic)
    is_precomputed_str = "precomputed=$(is_precomputed(h))"
    print(io, summary(h), "(",  "", is_precomputed_str, ")")
end

function precompute!(h::LM_CutHeuristic, domain::Domain, state::State, spec::Specification) 
    h.statics = infer_static_fluents(domain)
    h.graph = build_planning_graph(domain, state, spec; statics=h.statics)
    return h
end

function compute(h::LM_CutHeuristic, domain::Domain, state::State, spec::Specification)
    output_val = 0

    #Extract all action costs into a dictionary
    c = get_action_cost_dict(h.graph.actions, spec)

    #Redefine the cost of the init and goal actions for later Justification Graph construction
    goal_compound = Compound(:goal, Term[])
    c[goal_compound] = 0

    init_compound = Compound(:init, Term[])
    c[init_compound] = 0

    #Fetch idxs to use later on
    init_idxs = findall(SymbolicPlanners.pgraph_init_idxs(h.graph, domain, state))
    #idx of the new goal fact (simply an used idx)
    #the idx of the new init fact is then goal_fact_idx + 1
    goal_fact_idx = length(h.graph.conditions) + 1

    #Calculate relaxed costs of facts (delta) and the hmax value (goal_cost)
    delta, _, goal_idx, goal_cost = relaxed_pgraph_search(domain, state, spec, maximum, h.graph, c)


    #End unreachable searches early
    if goal_cost == Inf32
        return Inf32
    end

    while goal_cost != 0
        #Find the most expenseive precondition - the "supporter" for each action
        supporters = find_supporters(h.graph, delta)

        #Construct the backward and forward justification graphs
        forward_JG, backward_JG = build_JGs(supporters, h.graph, goal_idx, goal_fact_idx, init_idxs, c)

        #Find the N* partition
        n_star = find_nstar(backward_JG, goal_fact_idx)

        #Find N₀ partition and landmarks
        _, Lm, mi = find_nzero(forward_JG, n_star, c, goal_fact_idx)
        
        #Update heuristic value and cost function
        output_val += mi
        for o in Lm
            c[o] -= mi
        end
        
        delta, _, goal_idx, goal_cost = relaxed_pgraph_search(domain, state, spec, maximum, h.graph, c)
    end 
    return output_val
end

"Finds the most expenseive precondition - the supporter, for each action"
function find_supporters(graph, delta)
    supporters = Int[]
    for (act_idx, act) in enumerate(graph.actions)
        if length(act.preconds) == 1
            push!(supporters, graph.act_parents[act_idx][1][1])
        else
            curr_supp = argmax(maximum ∘ Base.Fix1(getindex, delta) , graph.act_parents[act_idx])

            push!(supporters, first(curr_supp))
        end
    end
    return supporters
end

"Builds forward and backward justification graphs"
function build_JGs(supporters, graph, goal_idx, goal_fact_idx, init_idxs, c)
    forward_JG = [Tuple{Term, Vector{Int}}[] for _ in 1:length(graph.conditions)+2]
    backward_JG = [Int[] for _ in 1:length(graph.conditions)+1]

    for (act_idx, act) in enumerate(graph.actions)
        act_children = graph.act_children[act_idx]
        supporter = supporters[act_idx]
        edge_ends = Int[]
        cost = c[act.term]

        for c_idx in act_children
            push!(edge_ends, c_idx)
            if cost == 0
                push!(backward_JG[c_idx], supporter)
            end
        end
        
        push!(forward_JG[supporter], (act.term, edge_ends))
    end
    #Add an extra edge to mark the goal action
    push!(backward_JG[goal_fact_idx], supporters[goal_idx])

    #Add edges from the arbitrary initial fact to the initial facts of the state
    init_compound = Compound(:init, Term[])
    push!(forward_JG[goal_fact_idx+1], (init_compound, init_idxs))

    return forward_JG, backward_JG
end

"Finds the N* partition in the justification graph"
function find_nstar(JG, goal_fact_idx)
    n_star = Set{Int}()
    q = Queue{Int}()
    enqueue!(q, goal_fact_idx)

    while !isempty(q)
        fact = dequeue!(q)
        edge_starts = JG[fact]

        for e in edge_starts
            if !(e in n_star)
                push!(n_star, e)
                enqueue!(q, e)
            end
        end
    end
    return n_star
end

"Finds the n_zero partition, the landmarks and the minimal landmark"
function find_nzero(JG, n_star, c, goal_fact_idx)
    
    n_zero = Set{Int}()
    q = Queue{Int}()

    mi = typemax(Float32)
    Lm = Set{Term}()
    
    #for init_idx in init_idxs
    #    push!(n_zero, init_idx)
    #    enqueue!(q, init_idx)
    #end
    push!(n_zero, goal_fact_idx+1)
    enqueue!(q, goal_fact_idx+1)

    
    while !isempty(q)
        fact = dequeue!(q)
        tups = JG[fact]
        for tup in tups
            act_term, edge_ends = tup
            for edge_end in edge_ends

                #Expand n_zero if explored fact is suitable           
                if !(edge_end in n_star) && !(edge_end in n_zero)
                    enqueue!(q,edge_end)
                    push!(n_zero, edge_end)
                #else check if the given fact is a landmark
                elseif edge_end in n_star
                    push!(Lm, act_term)
                    if (c[act_term] < mi)
                        mi = c[act_term]
                    end
                end
            end
        end
    end
    return n_zero, Lm, mi
end

function get_action_cost_dict(O, spec)
    c = Dict{Term, Float32}()
    if has_action_cost(spec)
        for o in O
            c[o.term] = get_action_cost(spec, o)
        end
    else
        for o in O            
            c[o.term] = 1
        end    
    end
    return c
end

"A variant of the relaxed_pgraph_search function which allows costs of actions to be passed in argument"
function relaxed_pgraph_search(domain::Domain, state::State, spec::Specification,
    accum_op::Function, graph::PlanningGraph, input_costs::Dict{Term, Float32})
    # Initialize fact costs, precondition flags,  etc.
    n_actions = length(graph.actions)
    n_conds = length(graph.conditions)
    dists = fill(Inf32, n_conds) # Fact distances
    costs = fill(Inf32, n_conds) # Fact costs
    achievers = fill(-1, n_conds) # Fact achievers
    condflags = [(UInt(1) << length(a.preconds)) - 1 for a in graph.actions]

    # Set up initial facts and priority queue
    init_idxs = pgraph_init_idxs(graph, domain, state)
    dists[init_idxs] .= 0
    costs[init_idxs] .= 0
    queue = PriorityQueue{Int,Float32}(i => 0 for i in findall(init_idxs))

    # Perform Djikstra / uniform-cost search until goals are reached
    goal_idx, goal_cost = nothing, Inf32
    first_goal_idx = n_actions - graph.n_goals
    while !isempty(queue) && isnothing(goal_idx)
        # Dequeue nearest fact/condition
        cond_idx = dequeue!(queue)
        # Iterate over child actions
        for (act_idx, precond_idx) in graph.cond_children[cond_idx]
            # Check if goal action is reached
            is_goal = act_idx > first_goal_idx
            # Skip actions with no children
            !is_goal && isempty(graph.act_children[act_idx]) && continue
            # Skip actions already achieved
            condflags[act_idx] === UInt(0) && continue
            # Set precondition flag for unachieved actions
            condflags[act_idx] &= ~(UInt(1) << (precond_idx - 1))
            # Continue if preconditions for action are not fully satisfied
            condflags[act_idx] != UInt(0) && continue
            # Compute path cost and distance of achieved action or axiom
            act_parents = graph.act_parents[act_idx]
            is_axiom = act_idx <= graph.n_axioms
            if is_axiom # Axiom cost is the max of parent costs
                next_cost = maximum(act_parents) do precond_parents
                    minimum(costs[p] for p in precond_parents)
                end
                next_dist = dists[cond_idx]
            elseif is_goal # Goal cost is the accumulation of parent costs
                next_cost = accum_op(act_parents) do precond_parents
                    minimum(costs[p] for p in precond_parents)
                end
            else # Look up action cost if specified, default to one otherwise
                path_cost = accum_op(act_parents) do precond_parents
                    minimum(costs[p] for p in precond_parents)
                end
                act_cost = input_costs[graph.actions[act_idx].term] 
                next_cost = path_cost + act_cost
                next_dist = accum_op === maximum && !has_action_cost(spec) ?
                            next_cost : dists[cond_idx] + 1
                
            end
            # Return goal index and goal cost if goal is reached
            if is_goal
                goal_idx, goal_cost = act_idx, next_cost
                break
            end
            # Place child conditions on queue
            act_children = graph.act_children[act_idx]
            for c_idx in act_children
                less_dist = next_dist < dists[c_idx]
                less_cost = next_cost < costs[c_idx]
                if !(less_dist || less_cost)
                    continue
                end
                if less_cost # Store new cost and achiever
                    costs[c_idx] = next_cost
                    achievers[c_idx] = act_idx
                end
                if !(c_idx in keys(queue)) # Enqueue new conditions
                    enqueue!(queue, c_idx, next_dist)
                elseif less_dist # Adjust distances
                    queue[c_idx] = next_dist
                    dists[c_idx] = next_dist
                end
            end
        end
    end

    # Return fact costs, achievers, goal index and cost
    return costs, achievers, goal_idx, goal_cost
end
