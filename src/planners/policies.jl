## Policy-based solutions ##
export PolicySolution, PolicyValueSolution
export best_action, rand_action, rollout

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
        state = transition(domain, state, act)
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
@kwdef mutable struct PolicyValueSolution <: PolicySolution
    V::Dict{UInt64,Float64} = Dict()
    Q::Dict{UInt64,Dict{Term,Float64}} = Dict()
    action_noise::Float64 = 0.0
end

best_action(sol::PolicyValueSolution, state::State) =
    argmax(sol.Q[hash(state)])

function rand_action(rng::AbstractRNG, sol::PolicyValueSolution, state::State)
    if sol.action_noise == 0 return best_action(sol, state) end
    actions = collect(keys(sol.Q[hash(state)]))
    probs = softmax(values(sol.Q[hash(state)]) ./ sol.action_noise)
    return sample(rng, actions, weights(probs))
end
