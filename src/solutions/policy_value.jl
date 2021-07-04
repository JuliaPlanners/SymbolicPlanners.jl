"Policy solution where values and Q-values are directly stored."
@kwdef struct PolicyValue <: PolicySolution
    V::Dict{UInt64,Float64} = Dict()
    Q::Dict{UInt64,Dict{Term,Float64}} = Dict()
end

get_action(sol::PolicyValue, state::State) =
    best_action(sol, state)
best_action(sol::PolicyValue, state::State) =
    argmax(sol.Q[hash(state)])
rand_action(sol::PolicyValue, state::State) =
    best_action(sol, state)
get_value(sol::PolicyValue, state::State) =
    sol.V[hash(state)]
get_value(sol::PolicyValue, state::State, action::Term) =
    sol.Q[hash(state)][action]
get_action_values(sol::PolicyValue, state::State) =
    pairs(sol.Q[hash(state)])
