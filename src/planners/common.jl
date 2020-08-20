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

"Extract plan from current state and back-pointers."
function extract_plan(state_hash::UInt, parents::Dict{UInt,Tuple{UInt,Term}})
    plan = Term[]
    while state_hash in keys(parents)
        state_hash, act = parents[state_hash]
        pushfirst!(plan, act)
    end
    return plan
end

function extract_plan(state_hash::UInt, state_dict::Dict{UInt,State},
                          parents::Dict{UInt,Tuple{UInt,Term}})
    plan, traj = Term[], State[state_dict[state_hash]]
    while state_hash in keys(parents)
        state_hash, act = parents[state_hash]
        pushfirst!(plan, act)
        pushfirst!(traj, state_dict[state_hash])
    end
    return plan, traj
end
