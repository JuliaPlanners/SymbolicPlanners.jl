## Utilities and solutions for path search algorithms ##
export PathSearchSolution, BiPathSearchSolution, AbstractPathSearchSolution

"""
$(TYPEDEF)

Abstract solution type for search-based planners.
"""
abstract type AbstractPathSearchSolution <: OrderedSolution end

function Base.show(io::IO, sol::AbstractPathSearchSolution)
    print(io, typeof(sol), "(", repr(sol.status), ", ", repr(sol.plan), ")")
end

function Base.show(io::IO, ::MIME"text/plain", sol::AbstractPathSearchSolution)
    println(io, typeof(sol))
    println(io, "  status: ", sol.status)
    println(io, "  plan: ", summary(sol.plan))
    n_lines, _ = displaysize(io)
    n_lines -= 5
    if length(sol.plan) > n_lines
        for act in sol.plan[1:(n_lines÷2-1)]
            println(io, "    ", write_pddl(act))
        end
        println(io, "    ", "⋮")
        for act in sol.plan[end-(n_lines÷2)+1:end]
            println(io, "    ", write_pddl(act))
        end
    else
        for act in sol.plan
            println(io, "    ", write_pddl(act))
        end
    end
    if !isnothing(sol.trajectory) && !isempty(sol.trajectory)
        print(io, "  trajectory: ", summary(sol.trajectory))
    end
end

Base.iterate(sol::AbstractPathSearchSolution) = iterate(sol.plan)
Base.iterate(sol::AbstractPathSearchSolution, istate) = iterate(sol.plan, istate)
Base.getindex(sol::AbstractPathSearchSolution, i::Int) = getindex(sol.plan, i)
Base.length(sol::AbstractPathSearchSolution) = length(sol.plan)

get_action(sol::AbstractPathSearchSolution, t::Int) = sol.plan[t]

function get_action(sol::AbstractPathSearchSolution, state::State)
    idx = findfirst(==(state), sol.trajectory)
    if isnothing(idx) || idx == length(sol.trajectory)
        return missing
    else
        return sol.plan[idx]
    end
end

function get_action(sol::AbstractPathSearchSolution, t::Int, state::State)
    return isnothing(sol.trajectory) ?
        get_action(sol, t) : get_action(sol, state)
end

best_action(sol::AbstractPathSearchSolution, state::State) =
    get_action(sol, state)
rand_action(sol::AbstractPathSearchSolution, state::State) =
    get_action(sol, state)

function get_action_probs(sol::AbstractPathSearchSolution, state::State)
    act = get_action(sol, state)
    return ismissing(act) ? Dict() : Dict(act => 1.0)
end

get_action_prob(sol::AbstractPathSearchSolution, state::State, action::Term) =
    action == best_action(sol, state) ? 1.0 : 0.0

"""
    PathNode(id::UInt, state::State, path_cost::Float32,
             parent_id::Union{UInt,Nothing},
             parent_action::Union{Term,Nothing})

Representation of search node with a backpointer, used by search-based planners.
"""
@auto_hash_equals mutable struct PathNode{S<:State}
    id::UInt
    state::S
    path_cost::Float32
    parent_id::Union{UInt,Nothing}
    parent_action::Union{Term,Nothing}
end

PathNode(id, state::S, path_cost, parent_id, parent_action) where {S} =
    PathNode{S}(id, state, Float32(path_cost), parent_id, parent_action)
PathNode(id, state::S, path_cost) where {S} =
    PathNode{S}(id, state, Float32(path_cost), nothing, nothing)

function reconstruct(node_id::UInt, search_tree::Dict{UInt,PathNode{S}}) where S
    plan, traj = Term[], S[]
    while node_id in keys(search_tree)
        node = search_tree[node_id]
        pushfirst!(traj, node.state)
        if node.parent_id === nothing break end
        pushfirst!(plan, node.parent_action)
        node_id = node.parent_id
    end
    return plan, traj
end

"""
    PathSearchSolution(status, plan)
    PathSearchSolution(status, plan, trajectory)
    PathSearchSolution(status, plan, trajectory, expanded,
                       search_tree, search_frontier, search_order)

Solution type for search-based planners that produce fully ordered plans.

# Fields

$(FIELDS)
"""
@auto_hash_equals mutable struct PathSearchSolution{
    S <: State, T
} <: AbstractPathSearchSolution
    "Status of the returned solution."
    status::Symbol
    "Sequence of actions that reach the goal. May be partial / incomplete."
    plan::Vector{Term}
    "Trajectory of states that will be traversed while following the plan."
    trajectory::Union{Vector{S},Nothing}
    "Number of nodes expanded during search."
    expanded::Int
    "Tree of [`PathNode`](@ref)s expanded or evaluated during search."
    search_tree::Union{Dict{UInt,PathNode{S}},Nothing}
    "Frontier of yet-to-be-expanded search nodes (stored as references)."
    search_frontier::T
    "Order of nodes expanded during search (stored as references)."
    search_order::Vector{UInt}
end

PathSearchSolution(status::Symbol, plan) =
    PathSearchSolution(status, convert(Vector{Term}, plan), State[],
                       -1, nothing, nothing, UInt[])
PathSearchSolution(status::Symbol, plan, trajectory) =
    PathSearchSolution(status, convert(Vector{Term}, plan), trajectory,
                       -1, nothing, nothing, UInt[])

function Base.copy(sol::PathSearchSolution)
    plan = copy(sol.plan)
    trajectory = isnothing(sol.trajectory) ?
        nothing : copy(sol.trajectory)
    search_tree = isnothing(sol.search_tree) ?
        nothing : copy(sol.search_tree)
    search_frontier = isnothing(sol.search_frontier) ?
        nothing : copy(sol.search_frontier)
    search_order = copy(sol.search_order)
    return PathSearchSolution(sol.status, plan, trajectory, sol.expanded,
                              search_tree, search_frontier, search_order)
end

function Base.show(io::IO, m::MIME"text/plain", sol::PathSearchSolution)
    # Invoke call to Base.show for AbstractPathSearchSolution
    invoke(show, Tuple{IO, typeof(m), AbstractPathSearchSolution}, io, m, sol)
    # Print search information if present
    if !isnothing(sol.search_tree)
        print(io, "\n  expanded: ", sol.expanded)
        print(io, "\n  search_tree: ", summary(sol.search_tree))
        print(io, "\n  search_frontier: ", summary(sol.search_frontier))
        if !isempty(sol.search_order)
            print(io, "\n  search_order: ", summary(sol.search_order))
        end
    end
end

"""
    BiPathSearchSolution(status, plan)
    BiPathSearchSolution(status, plan, trajectory)
    BiPathSearchSolution(status, plan, trajectory, expanded,
                         f_search_tree, f_frontier, f_expanded, f_trajectory,
                         b_search_tree, b_frontier, b_expanded, b_trajectory)

Solution type for bidirectional search-based planners.

# Fields

$(FIELDS)
"""
mutable struct BiPathSearchSolution{S<:State,T} <: AbstractPathSearchSolution
    "Status of the returned solution."
    status::Symbol
    "Sequence of actions that reach the goal. May be partial / incomplete."
    plan::Vector{Term}
    "Trajectory of states that will be traversed while following the plan."
    trajectory::Union{Vector{S},Nothing}
    "Number of nodes expanded during search."
    expanded::Int
    "Forward search tree."
    f_search_tree::Union{Dict{UInt,PathNode{S}},Nothing}
    "Forward search frontier."
    f_frontier::T
    "Number of nodes expanded via forward search."
    f_expanded::Int
    "Trajectory of states returned by forward search."
    f_trajectory::Union{Vector{S},Nothing}
    "Backward search tree."
    b_search_tree::Union{Dict{UInt,PathNode{S}},Nothing}
    "Backward search frontier."
    b_frontier::T
    "Number of nodes expanded via backward search."
    b_expanded::Int
    "Trajectory of states returned by backward search."
    b_trajectory::Union{Vector{S},Nothing}
end

BiPathSearchSolution(status::Symbol, plan) =
    BiPathSearchSolution(status, plan, State[], -1,
                         nothing, nothing, -1, nothing,
                         nothing, nothing, -1, nothing)
BiPathSearchSolution(status::Symbol, plan, trajectory) =
    BiPathSearchSolution(status, plan, trajectory, -1,
                         nothing, nothing, -1, nothing,
                         nothing, nothing, -1, nothing)

function Base.copy(sol::BiPathSearchSolution)
    fields = map(fieldnames(BiPathSearchSolution)) do field 
        x = getfield(sol, field)
        (x isa Symbol || isnothing(x)) ? x : copy(x)
    end
    return BiPathSearchSolution(fields...)
end

function Base.show(io::IO, m::MIME"text/plain", sol::BiPathSearchSolution)
    # Invoke call to Base.show for AbstractPathSearchSolution
    invoke(show, Tuple{IO, typeof(m), AbstractPathSearchSolution}, io, m, sol)
    # Print nodes expanded if present
    if sol.expanded >= 0
        print(io, "\n  expanded: ", sol.expanded)
    end
    # Print forward search information if present
    if !isnothing(sol.f_search_tree)
        print(io, "\n  f_search_tree: ", summary(sol.f_search_tree))
        print(io, "\n  f_frontier: ", summary(sol.f_frontier))
        print(io, "\n  f_expanded: ", sol.f_expanded)
        print(io, "\n  f_trajectory: ", summary(sol.f_trajectory))
    end
    # Print backward search information if present
    if !isnothing(sol.b_search_tree)
        print(io, "\n  b_search_tree: ", summary(sol.b_search_tree))
        print(io, "\n  b_frontier: ", summary(sol.b_frontier))
        print(io, "\n  b_expanded: ", sol.b_expanded)
        print(io, "\n  b_trajectory: ", summary(sol.b_trajectory))
    end
end

"Dequeue a key according to a Boltzmann distribution over priority values."
function prob_dequeue!(queue::PriorityQueue, temperature::Float64)
    if temperature == 0.0 return dequeue!(queue) end
    key, _ = prob_peek(queue, temperature)
    delete!(queue, key)
    return key
end

"Return a key according to a Boltzmann distribution over priority values."
function prob_peek(queue::PriorityQueue, temperature::Float64)
    if temperature == 0.0 return peek(queue) end
    _, min_priority = peek(queue)
    min_weight = first(min_priority)
    best_key, best_weight, best_priority = nothing, -Inf, nothing
    # Use Gumbel-Max reservoir sampling
    for (key, priority) in queue.xs
        weight = (min_weight - first(priority)) / temperature
        weight += randgumbel()
        if weight > best_weight
            best_weight = weight
            best_key = key
            best_priority = priority
        end
    end
    return best_key => best_priority
end
