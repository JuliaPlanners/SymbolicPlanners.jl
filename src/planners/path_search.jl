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
    if sol.status == :failure # Return no-op if goal is unreachable
        return convert(Term, PDDL.no_op)
    end
    idx = findfirst(==(state), sol.trajectory)
    if isnothing(idx) # Return missing if no corresponding state found
        return missing
    elseif idx == length(sol.trajectory) # Return no-op if goal is reached
        return sol.status == :success ? convert(Term, PDDL.no_op) : missing
    else # Return action corresponding to state
        return sol.plan[idx]
    end
end

function get_action(sol::AbstractPathSearchSolution, t::Int, state::State)
    return isnothing(sol.trajectory) || sol.trajectory[t] == state ?
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

"Linked list of parent or child node references."
@auto_hash_equals mutable struct LinkedNodeRef
    id::UInt
    action::Union{Term,Nothing}
    next::Union{LinkedNodeRef,Nothing}
end

LinkedNodeRef(id) = LinkedNodeRef(id, nothing, nothing)
LinkedNodeRef(id, action) = LinkedNodeRef(id, action, nothing)

Base.copy(ref::LinkedNodeRef) = LinkedNodeRef(ref.id, ref.action, ref.next)

function Base.unique(ref::LinkedNodeRef)
    unique_keys = Set{Tuple{UInt, Union{Term,Nothing}}}()
    iter = ref
    while !isnothing(iter)
        push!(unique_keys, (iter.id, iter.action))
        iter = iter.next
    end
    new_ref = nothing
    iter = ref
    while !isnothing(iter)
        if (iter.id, iter.action) in unique_keys
            new_ref = LinkedNodeRef(iter.id, iter.action, new_ref)
            delete!(unique_keys, (iter.id, iter.action))
        end
        iter = iter.next
    end
    return new_ref
end

"""
    PathNode(id::UInt, state::State, path_cost::Float32,
             parent = nothing, child = nothing)

Representation of search node with optional parent and child pointers, used
by search-based planners. One or more parents or children may be stored
as a linked list using the `LinkedNodeRef` data type.
"""
@auto_hash_equals mutable struct PathNode{S <: State}
    id::UInt
    state::S
    path_cost::Float32
    parent::Union{LinkedNodeRef,Nothing}
    child::Union{LinkedNodeRef,Nothing}
end

function PathNode{S}(id, state::S, path_cost,
                     parent = nothing) where {S <: State}
    return PathNode{S}(id, state, Float32(path_cost), parent, nothing)
end

function PathNode(id, state::S, path_cost,
                  parent = nothing, child = nothing) where {S}
    return PathNode{S}(id, state, Float32(path_cost), parent, child)
end

function Base.copy(node::PathNode{S}) where {S}
    parent = isnothing(node.parent) ? nothing : copy(node.parent)
    child = isnothing(node.child) ? nothing : copy(node.child)
    return PathNode{S}(node.id, node.state, node.path_cost, parent, child)
end

# function Base.hash(node::PathNode, h::UInt)
#     h = hash(node.id, h)
#     h = hash(node.state, h)
#     h = hash(node.path_cost, h)
#     h = isnothing(node.parent) ? h : hash(node.parent, h)
#     h = isnothing(node.child) ? h : hash(node.child, h)
#     return h
# end

# # Avoid recursive hashing and equality checking for `LinkedNodeRef`s
# function Base.hash(ref::LinkedNodeRef{T}, h::UInt) where {T <: PathNode}
#     h = hash(ref.node.id, h)
#     h = hash(ref.action, h)
#     h = isnothing(ref.next) ? h : hash(ref.next, h)
#     return h
# end

# function Base.:(==)(
#     ref1::LinkedNodeRef{T}, ref2::LinkedNodeRef{U}
# ) where {T <: PathNode, U <: PathNode}
#     T == U || return false
#     ref1.node.id == ref2.node.id || return false
#     typeof(ref1.action) == typeof(ref2.action) || return false
#     ref1.action == ref2.action || return false
#     typeof(ref1.next) == typeof(ref2.next) || return false
#     isnothing(ref1.next) && isnothing(ref2.next) && return true
#     return ref1.next == ref2.next
# end

# function Base.show(io::IO, ref::LinkedNodeRef{T}) where {T <: PathNode}
#     if isnothing(ref.next)
#         print(io, typeof(ref), "(", repr(ref.node.id), ", ", repr(ref.action), ")")
#     else
#         print(io, typeof(ref), "(", repr(ref.node.id), ", ", repr(ref.action), ", ", "...")
#     end
# end

# "Reconstructs a plan and trajectory from the start node to the provided node."
# function reconstruct(node::PathNode{S}) where {S}
#     plan, traj = Term[], S[node.state]
#     while !isnothing(node.parent) && !isnothing(node.parent.action)
#         pushfirst!(plan, node.parent.action)
#         node = node.parent.node
#         pushfirst!(traj, node.state)
#     end
#     return plan, traj
# end

"Reconstructs a plan and trajectory from the start node to the provided node."
function reconstruct(node_id::UInt, search_tree::Dict{UInt,PathNode{S}}) where S
    plan, trajectory = Term[], S[]
    node = get(search_tree, node_id, nothing)
    while !isnothing(node)
        pushfirst!(trajectory, node.state)
        (isnothing(node.parent) || isnothing(node.parent.action)) && break
        pushfirst!(plan, node.parent.action)
        node = get(search_tree, node.parent.id, nothing)
    end
    return plan, trajectory
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
        nothing : Dict(key => copy(val) for (key, val) in sol.search_tree)
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
    return isnothing(best_key) ? peek(queue) : best_key => best_priority
end
