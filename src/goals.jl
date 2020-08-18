export GoalSpec

"Goal specification for a planning problem."
@kwdef struct GoalSpec
    goals::Vector{Term} = Term[] # Goal terms to be satisfied
    metric::Union{Term,Nothing} = nothing # Metric to be minimized
    constraints::Vector{Term} = Term[] # Trajectory constraints
    GoalSpec(goals, metric, constraints) =
        new(flatten_conjs(goals), metric, flatten_conjs(constraints))
end

# Constructors
GoalSpec(goals::Vector{<:Term}) = GoalSpec(goals=goals)
GoalSpec(goal::Term) = GoalSpec(goals=flatten_conjs([goal]))
function GoalSpec(problem::Problem)
    goals = flatten_conjs(problem.goal)
    sign, metric = problem.metric
    if sign > 0 metric = Compound(:-, [metric]) end
    return GoalSpec(goals=goals, metric=metric)
end

# Conversions
Base.convert(::Type{GoalSpec}, goals::Vector{<:Term}) = GoalSpec(goals)
Base.convert(::Type{GoalSpec}, goal::Term) = GoalSpec(goal)
Base.convert(::Type{GoalSpec}, problem::Problem) = GoalSpec(problem)

# Hashing and equality
Base.hash(gs::GoalSpec, h::UInt) =
    hash(gs.constraints, hash(gs.metric, hash(Set(gs.goals), h)))
Base.:(==)(gs1::GoalSpec, gs2::GoalSpec) = Set(gs1.goals) == Set(gs2.goals) &&
    gs1.metric == gs2.metric && Set(gs1.constraints) == Set(gs2.constraints)
