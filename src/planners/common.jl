"Reconstruct plan from current state and back-pointers."
function reconstruct_plan(state_hash::UInt, state_dict::Dict{UInt,State},
                          parents::Dict{UInt,Tuple{UInt,Term}})
    plan, traj = Term[], State[state_dict[state_hash]]
    while state_hash in keys(parents)
        state_hash, act = parents[state_hash]
        pushfirst!(plan, act)
        pushfirst!(traj, state_dict[state_hash])
    end
    return plan, traj
end
