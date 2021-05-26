export SearchSolution

## Search utilities ##

mutable struct SearchNode
    state::State
    path_cost::Float64
    parent_hash::Union{UInt,Nothing}
    parent_action::Union{Term,Nothing}
end

SearchNode(state, path_cost) = SearchNode(state, path_cost, nothing, nothing)

const SearchTree = Dict{UInt,SearchNode}

function reconstruct(state_hash::UInt, search_tree::SearchTree)
    plan, traj = Term[], State[search_tree[state_hash].state]
    while state_hash in keys(search_tree)
        node = search_tree[state_hash]
        if node.parent_hash === nothing break end
        pushfirst!(plan, node.parent_action)
        pushfirst!(traj, node.state)
        state_hash = node.parent_hash
    end
    return plan, traj
end

## Common solution types ##

"Basic solution type that stores the computed plan and state trajectory."
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
