"Basic solution type that stores the computed plan and state trajectory."
mutable struct BasicSolution <: OrderedSolution
    plan::Vector{Term}
    trajectory::Vector{State}
    BasicSolution(plan) = new(plan)
    BasicSolution(plan, trajectory) = new(plan, trajectory)
end

Base.iterate(sol::BasicSolution) = iterate(sol.plan)
Base.iterate(sol::BasicSolution, state) = iterate(sol.plan, state)
Base.getindex(sol::BasicSolution, i::Int) = getindex(sol.plan, i)
Base.length(sol::BasicSolution) = length(sol.plan)

## Search utilities ##

mutable struct SearchNode
    state::State
    path_cost::Float64
    parent_hash::Union{UInt,Nothing}
    parent_action::Union{Term,Nothing}
end

SearchNode(state, path_cost) = SearchNode(state, path_cost, nothing, nothing)

function reconstruct(state_hash::UInt, search_tree::Dict{UInt,SearchNode})
    plan, traj = Term[], State[search_tree[state_hash].state]
    while state_hash in keys(search_tree)
        node = search_tree[state_hash]
        if node.parent_action === nothing break end
        pushfirst!(plan, node.parent_action)
        pushfirst!(traj, node.state)
        state_hash = node.parent_hash
    end
    return plan, traj
end
