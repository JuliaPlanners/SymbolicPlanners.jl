export BackwardPlanner, BackwardGreedyPlanner, BackwardAStarPlanner
export ProbBackwardPlanner, ProbBackwardAStarPlanner

"Heuristic-guided best-first backward search."
@kwdef mutable struct BackwardPlanner{T <: Union{Nothing, Float64}} <: Planner
    heuristic::Heuristic = GoalCountHeuristic(:backward)
    search_noise::T = nothing # Amount of (Boltzmann) search noise
    g_mult::Float32 = 1.0 # Path cost multiplier
    h_mult::Float32 = 1.0 # Heuristic multiplier
    max_nodes::Int = typemax(Int) # Max search nodes before termination
    max_time::Float64 = Inf # Max time in seconds before timeout
    save_search::Bool = false # Flag to save search tree in solution
    save_search_order::Bool = false # Flag to save search order
end

@auto_hash BackwardPlanner
@auto_equals BackwardPlanner

BackwardPlanner(heuristic::Heuristic, search_noise::T, args...) where {T} =
    BackwardPlanner{T}(heuristic, search_noise, args...)

"Backward greedy search, with cycle checking."
BackwardGreedyPlanner(heuristic::Heuristic; kwargs...) =
    BackwardPlanner(;heuristic=heuristic, g_mult=0, kwargs...)

"Backward A* search."
BackwardAStarPlanner(heuristic::Heuristic; kwargs...) =
    BackwardPlanner(;heuristic=heuristic, kwargs...)

"Probabilistic backward best-first search planner."
const ProbBackwardPlanner = BackwardPlanner{Float64}

ProbBackwardPlanner(;search_noise=1.0, kwargs...) = 
    BackwardPlanner(;search_noise=search_noise, kwargs...)

"Probabilistic backward A* search."
ProbBackwardAStarPlanner(heuristic::Heuristic; search_noise=1.0, kwargs...) =
    BackwardPlanner(;heuristic=heuristic, search_noise=search_noise, kwargs...)

function Base.copy(p::BackwardPlanner)
    return BackwardPlanner(p.heuristic, p.search_noise,
                           p.g_mult, p.h_mult, p.max_nodes, p.max_time,
                           p.save_search, p.save_search_order)
end

function solve(planner::BackwardPlanner,
               domain::Domain, state::State, spec::Specification)
    @unpack h_mult, heuristic, save_search = planner
    # Precompute heuristic information
    precompute!(heuristic, domain, state, spec)
    # Simplify goal specification
    spec = simplify_goal(spec, domain, state)
    # Convert to backward search goal specification
    spec = BackwardSearchGoal(spec, state)
    state = goalstate(domain, PDDL.get_objtypes(state), get_goal_terms(spec))
    # Initialize search tree and priority queue
    node_id = hash(state)
    search_tree = Dict(node_id => PathNode(node_id, state, 0.0))
    est_cost::Float32 = h_mult * compute(heuristic, domain, state, spec)
    priority = (est_cost, est_cost, 0)
    queue = PriorityQueue(node_id => priority)
    search_order = UInt[]
    sol = PathSearchSolution(:in_progress, Term[], Vector{typeof(state)}(),
                             0, search_tree, queue, search_order)
    # Run the search
    sol = search!(sol, planner, domain, spec)
    # Return solution
    if save_search
        return sol
    elseif sol.status == :failure
        return NullSolution(sol.status)
    else
        return PathSearchSolution(sol.status, sol.plan, sol.trajectory)
    end
end

function search!(sol::PathSearchSolution, planner::BackwardPlanner,
                 domain::Domain, spec::BackwardSearchGoal)
    start_time = time()
    sol.expanded = 0
    queue, search_tree = sol.search_frontier, sol.search_tree
    while length(queue) > 0
        # Get state with lowest estimated cost to goal
        node_id = isnothing(planner.search_noise) ?
            dequeue!(queue) : prob_dequeue!(queue, planner.search_noise)
        node = search_tree[node_id]
        # Check search termination criteria
        if is_goal(spec, domain, node.state)
            sol.status = :success # Goal reached
        elseif sol.expanded >= planner.max_nodes
            sol.status = :max_nodes # Node budget reached
        elseif time() - start_time >= planner.max_time
            sol.status = :max_time # Time budget reached
        end
        if sol.status == :in_progress # Expand current node
            expand!(planner, node, search_tree, queue, domain, spec)
            sol.expanded += 1
            if planner.save_search && planner.save_search_order
                push!(sol.search_order, node_id)
            end
        else # Reconstruct plan and return solution
            sol.plan, sol.trajectory = reconstruct(node_id, search_tree)
            reverse!(sol.plan)
            reverse!(sol.trajectory)
            return sol
        end
    end
    sol.status = :failure
    return sol
end

function expand!(planner::BackwardPlanner, node::PathNode,
                 search_tree::Dict{UInt,<:PathNode}, queue::PriorityQueue,
                 domain::Domain, spec::BackwardSearchGoal)
    @unpack g_mult, h_mult, heuristic = planner
    state = node.state
    # Iterate over relevant actions
    for act in relevant(domain, state)
        # Regress (reverse-execute) the action
        next_state = regress(domain, state, act; check=false)
        # Add constraints to regression state
        add_constraints!(spec, domain, state)
        next_id = hash(next_state)
        # Compute path cost
        act_cost = get_cost(spec, domain, state, act, next_state)
        path_cost = node.path_cost + act_cost
        # Update path costs if new path is shorter
        next_node = get!(search_tree, next_id,
                         PathNode(next_id, next_state, Inf32))
        cost_diff = next_node.path_cost - path_cost
        if cost_diff > 0
            next_node.parent_id = node.id
            next_node.parent_action = act
            next_node.path_cost = path_cost
            # Update estimated cost from next state to start
            if !(next_id in keys(queue))
                h_val::Float32 = compute(heuristic, domain, next_state, spec)
                f_val::Float32 = g_mult * path_cost + h_mult * h_val
                priority = (f_val, h_val, length(search_tree))
                enqueue!(queue, next_id, priority)
            else
                f_val, h_val, n_nodes = queue[next_id]
                queue[next_id] = (f_val - cost_diff, h_val, n_nodes)
            end
        end
    end
end
