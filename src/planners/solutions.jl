export Solution, NullSolution
export OrderedSolution, SearchSolution
export PolicySolution, ValuePolicySolution
export best_action, rand_action, rollout

"Abstract solution type, which defines the interface for planner solutions."
abstract type Solution end

"Null solution that indicates no plan was found."
struct NullSolution <: Solution end

## Ordered solutions ##

"Abstract type for ordered planner solutions."
abstract type OrderedSolution <: Solution end

# Ordered solutions should support indexing and iteration over action terms
Base.iterate(::OrderedSolution) = error("Not implemented.")
Base.iterate(::OrderedSolution, iterstate) = error("Not implemented.")
Base.getindex(::OrderedSolution, ::Int) = error("Not implemented.")
Base.eltype(::Type{OrderedSolution}) = Term


## Search-based solutions ##

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
Base.iterate(sol::SearchSolution, iterstate) = iterate(sol.plan, iterstate)
Base.getindex(sol::SearchSolution, i::Int) = getindex(sol.plan, i)
Base.length(sol::SearchSolution) = length(sol.plan)

## Policy-based solutions ##

"Abstract type for policy-based solutions."
abstract type PolicySolution <: Solution end

"Returns the best action for the given state."
best_action(::PolicySolution, ::State) =
    error("Not implemented.")

"Samples an action according to the policy for the given state."
rand_action(::AbstractRNG, ::PolicySolution, ::State) =
    error("Not implemented.")
rand_action(sol::PolicySolution, state::State) =
    rand_action(Random.GLOBAL_RNG, sol, state)

"Rollout a policy from the given state."
function rollout(sol::PolicySolution,
                 state::State, domain::Domain, n_steps::Int)
    actions = Term[]
    trajectory = State[state]
    for t in 1:n_steps
        act = rand_action(sol, state)
        state = transition(domain, state, action)
        push!(actions, act)
        push!(trajectory, state)
    end
    return actions, trajectory
end

function rollout(sol::PolicySolution,
                 state::State, domain::Domain, goal_spec::GoalSpec)
    actions = Term[]
    trajectory = State[state]
    while !satisfy(goal_spec.goals, state, domain)[1]
        act = rand_action(sol, state)
        state = transition(domain, state, act)
        push!(actions, act)
        push!(trajectory, state)
    end
    return actions, trajectory
end

rollout(sol::PolicySolution, state::State, domain::Domain, goal) =
    rollout(sol, state, domain, GoalSpec(goal))

"Convert vector of scores to probabiities."
function softmax(scores)
    ws = exp.(scores .- maximum(scores))
    z = sum(ws)
    return isnan(z) ? ones(length(scores)) ./ length(scores) : ws ./ z
end

"Policy solution where values and q-values are directly stored in a hashtable."
@kwdef mutable struct ValuePolicySolution <: PolicySolution
    V::Dict{UInt64,Float64} = Dict()
    Q::Dict{UInt64,Dict{Term,Float64}} = Dict()
    action_noise::Float64 = 0.0
end

best_action(sol::ValuePolicySolution, state::State) =
    argmax(sol.Q[hash(state)])

function rand_action(rng::AbstractRNG, sol::ValuePolicySolution, state::State)
    if sol.action_noise == 0 return best_action(sol, state) end
    actions = collect(keys(sol.Q[hash(state)]))
    probs = softmax(values(sol.Q[hash(state)]) ./ sol.action_noise)
    return sample(rng, actions, weights(probs))
end
