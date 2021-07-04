export OrderedPlan

"Solution type for fully ordered plans."
struct OrderedPlan <: OrderedSolution
    plan::Vector{Term}
end

get_action(sol::OrderedPlan, t::Int) = sol.plan[t]

Base.iterate(sol::OrderedPlan) = iterate(sol.plan)
Base.iterate(sol::OrderedPlan, istate) = iterate(sol.plan, istate)
Base.getindex(sol::OrderedPlan, i::Int) = getindex(sol.plan, i)
Base.length(sol::OrderedPlan) = length(sol.plan)
