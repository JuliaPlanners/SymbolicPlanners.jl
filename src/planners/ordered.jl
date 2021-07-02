## Ordered solutions ##
export OrderedSolution, OrderedSearchSolution

"Abstract type for ordered planner solutions."
abstract type OrderedSolution <: Solution end

# Ordered solutions should support indexing and iteration over action terms
Base.iterate(::OrderedSolution) = error("Not implemented.")
Base.iterate(::OrderedSolution, iterstate) = error("Not implemented.")
Base.getindex(::OrderedSolution, ::Int) = error("Not implemented.")
Base.eltype(::Type{OrderedSolution}) = Term

mutable struct SearchNode
    id::UInt
    state::State
    path_cost::Float64
    parent_id::Union{UInt,Nothing}
    parent_action::Union{Term,Nothing}
end

SearchNode(id, state, path_cost) =
    SearchNode(id, state, path_cost, nothing, nothing)

const SearchTree = Dict{UInt,SearchNode}

function reconstruct(node_id::UInt, search_tree::SearchTree)
    plan, traj = Term[], State[search_tree[node_id].state]
    while node_id in keys(search_tree)
        node = search_tree[node_id]
        if node.parent_id === nothing break end
        pushfirst!(plan, node.parent_action)
        pushfirst!(traj, node.state)
        node_id = node.parent_id
    end
    return plan, traj
end

"Solution type for search-based planners that produce fully ordered plans."
mutable struct OrderedSearchSolution{T} <: OrderedSolution
    status::Symbol
    plan::Vector{Term}
    trajectory::Union{Vector{State},Nothing}
    search_tree::Union{SearchTree,Nothing}
    frontier::T
end

OrderedSearchSolution(status::Symbol, plan) =
    OrderedSearchSolution(status, plan, nothing, nothing, nothing)
OrderedSearchSolution(status::Symbol, plan, trajectory) =
    OrderedSearchSolution(status, plan, trajectory, nothing, nothing)

Base.iterate(sol::OrderedSearchSolution) =
    iterate(sol.plan)
Base.iterate(sol::OrderedSearchSolution, iterstate) =
    iterate(sol.plan, iterstate)
Base.getindex(sol::OrderedSearchSolution, i::Int) =
    getindex(sol.plan, i)
Base.length(sol::OrderedSearchSolution) =
    length(sol.plan)
