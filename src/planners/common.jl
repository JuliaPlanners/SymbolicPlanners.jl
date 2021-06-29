export SearchSolution

## Search utilities ##

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

## Common solution types ##

"Solution type for search-based planners that produce fully ordered plans."
mutable struct SearchSolution{T} <: OrderedSolution
    status::Symbol
    plan::Vector{Term}
    trajectory::Union{Vector{State},Nothing}
    search_tree::Union{SearchTree,Nothing}
    frontier::T
end

SearchSolution(status::Symbol, plan) =
    SearchSolution(status, plan, nothing, nothing, nothing)
SearchSolution(status::Symbol, plan, trajectory) =
    SearchSolution(status, plan, trajectory, nothing, nothing)

Base.iterate(sol::SearchSolution) = iterate(sol.plan)
Base.iterate(sol::SearchSolution, state) = iterate(sol.plan, state)
Base.getindex(sol::SearchSolution, i::Int) = getindex(sol.plan, i)
Base.length(sol::SearchSolution) = length(sol.plan)
