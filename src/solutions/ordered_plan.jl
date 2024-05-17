export OrderedPlan

"""
    OrderedPlan(plan::AbstractVector{<:Term})

Generic solution type for fully ordered plans.
"""
@auto_hash_equals struct OrderedPlan <: OrderedSolution
    plan::Vector{Term}
end

function Base.show(io::IO, ::MIME"text/plain", sol::OrderedPlan)
    indent = get(io, :indent, "")
    show_struct(io, sol; indent = indent, show_pddl_list=(:plan,))
end

Base.copy(sol::OrderedPlan) =
    OrderedPlan(copy(sol.plan))

get_action(sol::OrderedPlan, t::Int) = sol.plan[t]

Base.iterate(sol::OrderedPlan) = iterate(sol.plan)
Base.iterate(sol::OrderedPlan, istate) = iterate(sol.plan, istate)
Base.getindex(sol::OrderedPlan, i::Int) = getindex(sol.plan, i)
Base.length(sol::OrderedPlan) = length(sol.plan)
