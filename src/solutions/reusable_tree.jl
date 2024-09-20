export ReusableTreePolicy

"""
    ReusableTreePolicy(
        value_policy::PolicySolution,
        search_sol::PathSearchSolution,
        [goal_tree::Dict{UInt, PathNode}]
    )

The policy returned by [`RealTimeHeuristicSearch`](@ref), which stores a value
table in the nested `value_policy`, a forward search tree in `search_sol`, and
(when `reuse_paths` is `true`) a reusable `goal_tree` of cost-optimal paths
to the goal.

When taking actions at states along some stored cost-optimal path, actions
along that path are followed, as in Tree-Adaptive A* [1]. Otherwise, the highest
value action according to `value_policy` is returned, with ties broken by 
the (possibly incomplete) plan in `search_sol`.

[1] C. Hernández, X. Sun, S. Koenig, and P. Meseguer, "Tree Adaptive A*," 
AAMAS (2011), pp. 123–130. <https://dl.acm.org/doi/abs/10.5555/2030470.2030488>.
"""
@auto_hash_equals struct ReusableTreePolicy{
    S <: State, P <: PolicySolution, Q
} <: PolicySolution
    value_policy::P # Nested value policy
    search_sol::PathSearchSolution{S,Q} # Forward search solution
    goal_tree::Dict{UInt, PathNode{S}} # Reusable tree of cost-optimal paths
end

function ReusableTreePolicy(
    policy::P, search_sol::PathSearchSolution{S, Q} 
) where {S <: State, P <: PolicySolution, Q}
    ReusableTreePolicy{S}(policy, search_sol)
end

function ReusableTreePolicy{S}(
    policy::P, search_sol::PathSearchSolution{S, Q} 
) where {S <: State, P <: PolicySolution, Q}
    ReusableTreePolicy{S, P, Q}(policy, search_sol, Dict{UInt, PathNode{S}}())
end

function ReusableTreePolicy{S, P}(
    policy::P, search_sol::PathSearchSolution{S, Q}
) where {S <: State, P <: PolicySolution, Q}
    ReusableTreePolicy{S, P, Q}(policy, search_sol, Dict{UInt, PathNode{S}}())
end

function Base.copy(sol::ReusableTreePolicy)
    return ReusableTreePolicy(
        copy(sol.value_policy), copy(sol.search_sol), copy(sol.goal_tree)
    )
end

function Base.show(io::IO, ::MIME"text/plain", sol::ReusableTreePolicy)
    indent = get(io, :indent, "")
    show_struct(io, sol; indent = indent,
                show_fields = (:value_policy, :search_sol))
end

get_action(sol::ReusableTreePolicy, state::State) =
    best_action(sol, state)
rand_action(sol::ReusableTreePolicy, state::State) =
    best_action(sol, state)

function best_action(sol::ReusableTreePolicy, state::State)
    node = isempty(sol.goal_tree) ?
        nothing : get(sol.goal_tree, hash(state), nothing)
    if (!isnothing(node) && !isnothing(node.parent) &&
        !isnothing(node.parent.action))
        # Reuse cost-optimal path if it exists
        return node.parent.action
    else
        # Look up best action according to value policy
        best_act = best_action(sol.value_policy, state)
        ismissing(best_act) && return missing
        # Return if search solution agrees on best action
        search_act = get_action(sol.search_sol, state)
        (ismissing(search_act) || search_act === PDDL.no_op) && return best_act
        search_act == best_act && return best_act
        # Otherwise tie-break equally good actions in favor of search solution
        best_val = get_value(sol.value_policy, state, best_act)
        search_val = get_value(sol.value_policy, state, search_act)
        return best_val > search_val ? best_act : search_act
    end
end

has_values(sol::ReusableTreePolicy) =
    has_values(sol.value_policy)

get_value(sol::ReusableTreePolicy, state::State) =
    get_value(sol.value_policy, state)

get_value(sol::ReusableTreePolicy, state_id::UInt, default = nothing) =
    get_value(sol.value_policy, state_id, default)

get_value(sol::ReusableTreePolicy, state::State, action::Term) =
    get_value(sol.value_policy, state, action)

get_action_values(sol::ReusableTreePolicy, state::State) =
    get_action_values(sol.value_policy, state)

set_value!(sol::ReusableTreePolicy, state::State, val::Real) =
    set_value!(sol.value_policy, state, val)

set_value!(sol::ReusableTreePolicy, state_id::UInt, val::Real) =
    set_value!(sol.value_policy, state_id, val)

has_cached_value(sol::ReusableTreePolicy, state::State) =
    has_cached_value(sol.value_policy, state)

has_cached_value(sol::ReusableTreePolicy, state_id::UInt) =
    has_cached_value(sol.value_policy, state_id)

has_cached_value(sol::ReusableTreePolicy, state::State, action::Term) =
    has_cached_value(sol.value_policy, state, action)

has_cached_value(sol::ReusableTreePolicy, state_id::UInt, action::Term) =
    has_cached_value(sol.value_policy, state_id, action)

has_cached_action_values(sol::ReusableTreePolicy, state::State) =
    has_cached_action_values(sol.value_policy, state)

"Insert path that terminates at `node_id` from search tree into reusable tree."
function insert_path!(
    goal_tree::Dict{UInt, PathNode{S}}, search_tree::Dict{UInt, PathNode{S}},
    node_id::UInt, terminal_h_val::Float32
) where {S <: State}
    s_node = search_tree[node_id]
    terminal_f_val = s_node.path_cost + terminal_h_val
    g_node = get!(goal_tree, node_id) do
        # Insert new root/goal node to reusable tree
        PathNode{S}(node_id, s_node.state, terminal_h_val)
    end
    while !isnothing(s_node.parent) && !isnothing(s_node.parent.action)
        next_id = s_node.parent.id
        next_s_node = search_tree[next_id]
        g_node = get!(goal_tree, next_id) do
            # Insert new intermediate node to reusable tree
            h_val = terminal_f_val - next_s_node.path_cost
            parent_ref = LinkedNodeRef(node_id, s_node.parent.action)
            PathNode{S}(next_id, next_s_node.state, h_val, parent_ref)
        end
        node_id = next_id
        s_node = next_s_node
    end
    return goal_tree
end

function insert_path!(
    sol::ReusableTreePolicy{S}, search_tree::Dict{UInt, PathNode{S}},
    node_id::UInt, terminal_h_val::Float32
) where {S <: State} 
    insert_path!(sol.goal_tree, search_tree, node_id, terminal_h_val)
end

"""
    ReusableTreeGoal(spec::Specification, goal_tree::Dict{UInt, PathNode})

Wrapper goal specification that returns `true` if the undelying `spec` is 
satisfied, or if a node in the reusable `goal_tree` has been reached.
"""
struct ReusableTreeGoal{T <: Specification, U <: State} <: Specification
    spec::T
    goal_tree::Dict{UInt, PathNode{U}}
end

Base.hash(spec::ReusableTreeGoal, h::UInt) =
    hash(spec.goal_tree, hash(spec.spec, h))
Base.:(==)(s1::ReusableTreeGoal, s2::ReusableTreeGoal) =
    s1.goal_tree == s2.goal_tree && s1.spec == s2.spec

"""
    on_goal_path(spec, domain, state)

If `spec` is a `ReusableTreeGoal`, returns `true` if a node in the reusable tree
has been reached. Otherwise, returns `false`.
"""
on_goal_path(spec::Specification, domain::Domain, state::State) =
    false
on_goal_path(spec::ReusableTreeGoal, domain::Domain, state::State) =
    (!isempty(spec.goal_tree) && haskey(spec.goal_tree, hash(state)))

is_goal(spec::ReusableTreeGoal, domain::Domain, state::State) =
    is_goal(spec.spec, domain, state)

is_goal(spec::ReusableTreeGoal, domain::Domain, state::State, action) =
    is_goal(spec.spec, domain, state, action)

is_violated(spec::ReusableTreeGoal, domain::Domain, state::State) =
    is_violated(spec.spec, domain, state)
get_cost(spec::ReusableTreeGoal, domain::Domain, s1::State, a::Term, s2::State) =
    get_cost(spec.spec, domain, s1, a, s2)
get_reward(spec::ReusableTreeGoal, domain::Domain, s1::State, a::Term, s2::State) =
    get_reward(spec.spec, domain, s1, a, s2)
get_discount(spec::ReusableTreeGoal) =
    get_discount(spec.spec)
get_goal_terms(spec::ReusableTreeGoal) =
    get_goal_terms(spec.spec)

set_goal_terms(spec::ReusableTreeGoal, terms) =
    ReusableTreeGoal(set_goal_terms(spec.spec, terms), spec.goal_tree)

has_action_goal(spec::ReusableTreeGoal) = has_action_goal(spec.spec)
has_action_cost(spec::ReusableTreeGoal) = has_action_cost(spec.spec)
get_action_cost(spec::ReusableTreeGoal, action::Term) =
    get_action_cost(spec.spec, action)
