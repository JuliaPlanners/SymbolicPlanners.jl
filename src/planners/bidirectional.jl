export BidirectionalPlanner, BiGreedyPlanner, BiAStarPlanner

"""
    planner = BidirectionalPlanner(;
        forward::ForwardPlanner = ForwardPlanner(),
        backward::BackwardPlanner = BackwardPlanner(),
        max_nodes::Int = typemax(Int),
        max_time::Float64 = Inf,
        save_search::Bool = false
    )

A bi-directional planner which simulataneously runs a forward search from the
initial state and backward search from the goal, succeeding if either search is
successful, or if the search frontiers are detected to cross.

Frontier crossing is detected by checking whether the most recently expanded
forward node is subsumed by a node in the backward search frontier, or
vice versa. Subsumption means that the partial state represented by a backward
node is consistent with the complete state represented by forward node.

While the above procedure is not complete (i.e. some crossings will be missed),
it represents a trade-off between the cost of testing for subsumption and the 
benefit of detecting a crossing, in lieu of more sophisticated methods [1].

[1] V. Alcázar, S. Fernández, and D. Borrajo, "Analyzing the Impact of Partial
States on Duplicate Detection and Collision of Frontiers," ICAPS (2014),
https://doi.org/10.1609/icaps.v24i1.13677.

# Arguments

$(FIELDS)
"""
@kwdef mutable struct BidirectionalPlanner <: Planner
    "Forward search configuration."
    forward::ForwardPlanner = ForwardPlanner()
    "Forward search configuration."
    backward::BackwardPlanner = BackwardPlanner()
    "Maximum number of search nodes before termination."
    max_nodes::Int = typemax(Int)
    "Maximum time in seconds before planner times out."
    max_time::Float64 = Inf
    "Flag to save the search tree and frontier in the returned solution."
    save_search::Bool = false
end

@auto_hash BidirectionalPlanner
@auto_equals BidirectionalPlanner

function BidirectionalPlanner(
    f_heuristic::Heuristic, b_heuristic::Heuristic;
    max_nodes = typemax(Int64), max_time = Inf, save_search = false, kwargs...
) 
    BidirectionalPlanner(
        ForwardPlanner(
            heuristic= f_heuristic,
            max_nodes = max_nodes,
            max_time = max_time,
            save_search = save_search,
            kwargs...
        ),
        BackwardPlanner(
            heuristic=b_heuristic,
            max_nodes = max_nodes,
            max_time = max_time,
            save_search = save_search,
            kwargs...
        ),
        max_nodes, 
        max_time, 
        save_search
    )
end

"""
$(SIGNATURES)

Bidirectional greedy best-first search, where `f_heuristic` is the forward
search heuristic and `b_heuristic`` is the backward search heuristic. Options
specified as `kwargs` are shared by both the backward and forward search.
"""
BiGreedyPlanner(f_heuristic::Heuristic, b_heuristic::Heuristic;  kwargs...) = 
    BidirectionalPlanner(f_heuristic, b_heuristic; g_mult=0, kwargs...)

"""
$(SIGNATURES)

Bidirectional A* search, where `f_heuristic` is the forward search heuristic
and `b_heuristic`` is the backward search heuristic. Options specified as
`kwargs` are shared by both the backward and forward search.
"""
BiAStarPlanner(f_heuristic::Heuristic, b_heuristic::Heuristic; kwargs...) =
    BidirectionalPlanner(f_heuristic, b_heuristic; kwargs...)

function Base.copy(p::BidirectionalPlanner)
    return BidirectionalPlanner(copy(p.forward), copy(p.backward),
                                p.max_nodes, p.max_time, p.save_search)
end
    
function solve(planner::BidirectionalPlanner,
               domain::Domain, state::State, spec::Specification)
    # Simplify goal specification
    f_spec = simplify_goal(spec, domain, state)
    b_spec = BackwardSearchGoal(spec, state)
    # Precompute heuristic information
    precompute!(planner.forward.heuristic, domain, state, f_spec)
    precompute!(planner.backward.heuristic, domain, state, b_spec)
    # Initialize search queues and search solution
    f_search_tree, f_queue =
        init_forward(planner.forward, domain, state, f_spec)
    b_search_tree, b_queue =
        init_backward(planner.backward, domain, state, b_spec)
    sol = BiPathSearchSolution(:in_progress, Term[], nothing, 0,
                               f_search_tree, f_queue, 0, nothing,
                               b_search_tree, b_queue, 0, nothing)
    # Run the search
    sol = search!(sol, planner, domain, state, f_spec, b_spec)
    # Return solution
    if planner.save_search
        return sol
    elseif sol.status == :failure
        return NullSolution(sol.status)
    else
        return BiPathSearchSolution(sol.status, sol.plan, sol.trajectory)
    end
end

function init_forward(planner::ForwardPlanner,
                      domain::Domain, state::State, spec::Specification)
    @unpack h_mult, heuristic, save_search = planner
    node_id = hash(state)
    search_tree = Dict(node_id => PathNode(node_id, state, 0.0))
    est_cost::Float32 = h_mult * compute(heuristic, domain, state, spec)
    priority = (est_cost, est_cost, 0)
    queue = PriorityQueue(node_id => priority)
    return(search_tree, queue)
end

function init_backward(planner::BackwardPlanner,
                       domain::Domain, state::State, spec::Specification)
    @unpack h_mult, heuristic, save_search = planner
    spec = BackwardSearchGoal(spec, state)
    state = goalstate(domain, PDDL.get_objtypes(state), get_goal_terms(spec))
    # Initialize search tree and priority queue
    node_id = hash(state)
    search_tree = Dict(node_id => PathNode(node_id, state, 0.0))
    est_cost::Float32 = h_mult * compute(heuristic, domain, state, spec)
    priority = (est_cost, est_cost, 0)
    queue = PriorityQueue(node_id => priority)
    return(search_tree, queue)
end

function search!(sol::BiPathSearchSolution,  planner::BidirectionalPlanner,
                 domain::Domain, state::State, 
                 f_spec::Specification, b_spec::Specification)
    @unpack max_nodes, max_time = planner
    @unpack f_search_tree, b_search_tree = sol
    f_search_noise = planner.forward.search_noise
    b_search_noise = planner.backward.search_noise
    f_queue, b_queue = sol.f_frontier, sol.b_frontier
    sol.expanded, sol.f_expanded, sol.b_expanded = 0, 0, 0
    f_node_id, b_node_id = nothing, nothing
    f_reached, b_reached, crossed = false, false, false
    # Functions for detecting frontier crossing
    function find_f_in_b_queue(node)
        for b_id in keys(b_queue)
            issubset(b_search_tree[b_id].state, node.state) && return b_id
        end
        return nothing
    end        
    function find_b_in_f_queue(node)
        for f_id in keys(f_queue)
            issubset(node.state, f_search_tree[f_id].state) && return f_id
        end
        return nothing
    end        
    start_time = time()
    while !isempty(f_queue) || !isempty(b_queue)
        # Advance the forward search
        if !isempty(f_queue)
            f_node_id, _ = isnothing(f_search_noise) ?
                peek(f_queue) : prob_peek(f_queue, f_search_noise)
            f_node = f_search_tree[f_node_id]
            # Check if goal is reached
            if is_goal(f_spec, domain, f_node.state)
                f_reached = true; sol.status = :success; break
            end
            # Check if frontiers cross
            b_node_id = find_f_in_b_queue(f_node)
            if !isnothing(b_node_id)
                crossed = true; sol.status = :success; break
            end
            # Dequeue node          
            isnothing(f_search_noise) ?
                dequeue!(f_queue) : dequeue!(f_queue, f_node_id)
            # Expand node
            expand!(planner.forward, f_node,
                    f_search_tree, f_queue, domain, f_spec)
            sol.f_expanded += 1
            sol.expanded += 1
        end
         # Advance the backward search
        if !isempty(b_queue)
            b_node_id, _ = isnothing(b_search_noise) ?
                peek(b_queue) : prob_peek(b_queue, b_search_noise)
            b_node = b_search_tree[b_node_id]
            # Check if goal is reached
            if is_goal(b_spec, domain, b_node.state)
                b_reached = true; sol.status = :success; break
            end
            # Check if frontiers cross
            f_node_id = find_b_in_f_queue(b_node)
            if !isnothing(f_node_id)
                crossed = true; sol.status = :success; break
            end
            # Dequeue node          
            isnothing(b_search_noise) ?
                dequeue!(b_queue) : dequeue!(b_queue, b_node_id)
            # Expand node
            expand!(planner.backward, b_node,
                    b_search_tree, b_queue, domain, b_spec)
            sol.b_expanded += 1
            sol.expanded += 1
        end
        # Check if resource limits are exceeded
        if sol.expanded >= max_nodes
            sol.status = :max_nodes # Node budget reached
            break
        elseif time() - start_time >= max_time
            sol.status = :max_times # Time budget reached
            break
        end
    end
    # Reconstruct plan if one is found
    if sol.status == :in_progress # No solution found
        sol.status = :failure
    elseif f_reached
        sol.plan, sol.f_trajectory = reconstruct(f_node_id, f_search_tree)
        sol.trajectory = sol.f_trajectory
    elseif b_reached
        sol.plan, sol.b_trajectory = reconstruct(b_node_id, b_search_tree)
        sol.trajectory = simulate(StateRecorder(), domain, state, sol.plan)
    elseif crossed
        f_plan, sol.f_trajectory = reconstruct(f_node_id, f_search_tree)
        b_plan, sol.b_trajectory = reconstruct(b_node_id, b_search_tree)
        sol.plan = vcat(f_plan, reverse(b_plan))
        sol.trajectory = simulate(StateRecorder(), domain, state, sol.plan)
    end
    return sol
end

function refine!(
    sol::BiPathSearchSolution{S, T}, planner::BidirectionalPlanner,
    domain::Domain, state::State, spec::Specification
) where {S, T <: PriorityQueue}
    sol.status == :success && return sol
    sol.status = :in_progress
    f_spec = simplify_goal(spec, domain, state)
    b_spec = BackwardSearchGoal(spec, state)
    ensure_precomputed!(planner.forward.heuristic, domain, state, f_spec)
    ensure_precomputed!(planner.backward.heuristic, domain, state, b_spec)
    return search!(sol, planner, domain, state, f_spec, b_spec)
end
