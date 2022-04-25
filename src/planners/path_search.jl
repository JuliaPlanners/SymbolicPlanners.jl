## Utilities and solutions for path search algorithms ##
export PathSearchSolution

mutable struct PathNode{S<:State}
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

"Solution type for search-based planners that produce fully ordered plans."
mutable struct PathSearchSolution{S<:State,T} <: OrderedSolution
    status::Symbol
    plan::Vector{Term}
    trajectory::Union{Vector{S},Nothing}
    expanded::Int
    search_tree::Union{Dict{UInt,PathNode{S}},Nothing}
    frontier::T
end

PathSearchSolution(status::Symbol, plan) =
    PathSearchSolution(status, plan, nothing, -1, nothing, nothing)
PathSearchSolution(status::Symbol, plan, trajectory) =
    PathSearchSolution(status, plan, trajectory, -1, nothing, nothing)

get_action(sol::OrderedPlan, t::Int, state::State) = sol.plan[t]

Base.iterate(sol::PathSearchSolution) = iterate(sol.plan)
Base.iterate(sol::PathSearchSolution, istate) = iterate(sol.plan, istate)
Base.getindex(sol::PathSearchSolution, i::Int) = getindex(sol.plan, i)
Base.length(sol::PathSearchSolution) = length(sol.plan)
