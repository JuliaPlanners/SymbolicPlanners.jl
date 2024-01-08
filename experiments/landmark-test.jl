## Benchmarking experiments for planners on propositional domains ##
using PDDL, SymbolicPlanners, Test
using DataFrames, CSV, Dates, Statistics

println("Started")

DOMAIN_NAME = "blocksworld"
INSTANCE = 19
PROBLEM_SIZE_REGEX = r"\w+-(\d+)-(\d+)"

function get_problem_size(problem)
    m = match(PROBLEM_SIZE_REGEX, string(problem.name))
    return parse(Int, m.captures[1])
end

## Load domain and problem ##

domain_dir = joinpath(@__DIR__, "logical", DOMAIN_NAME)
domain = load_domain(joinpath(domain_dir, "domain.pddl"))
problem = load_problem(joinpath(domain_dir, "instance-$INSTANCE.pddl"))
# domain = load_domain(joinpath(@__DIR__, "logical", "freecell", "domain.pddl"))
# problem = load_problem(joinpath(@__DIR__, "logical", "freecell", "instance-26.pddl"))
# domain = load_domain(joinpath(@__DIR__, "logical", "grid", "domain.pddl"))
# problem = load_problem(joinpath(@__DIR__, "logical", "grid", "instance-1.pddl"))

state = initstate(domain, problem)
spec = Specification(problem)
planner = OrderedLandmarksPlanner()

## Experimental code ##

# g = compute_landmark_graph(domain, state, spec)

rg = compute_relaxed_landmark_graph(domain, state, spec)
approximate_reasonable_orders(rg.first, rg.second)

import SymbolicPlanners.LandmarkNode, SymbolicPlanners.EdgeType, SymbolicPlanners.FactPair

function factpair_to_term(factpair::FactPair) :: Term
    effect::Term = rg.second.planning_graph.conditions[factpair.var]
    if factpair.value == 0
        effect = Compound(:not, [effect])
    end
    return effect
end

node_lookup::Dict{LandmarkNode, Int} = Dict(map(reverse, enumerate(rg.first.nodes)))
for node::Tuple{Int, LandmarkNode} in enumerate(rg.first.nodes)
    println("Landmark $(node[1]) ($(factpair_to_term(first(node[2].landmark.facts))))")
    for child::Pair{LandmarkNode, EdgeType} in node[2].children
        println("    Edge to $(node_lookup[child.first]) - $(child.second)")
    end
end
println()

## Verification ##

println("Verifying interpreted")
@elapsed sol = planner(domain, state, spec)
@test is_goal(spec, domain, sol.trajectory[end])

println()
println("Solution steps")
not_reached::Set{Int} = Set(range(1, length = length(rg.first.nodes)))
active::Set{Int} = Set()
for (i::Int, s::GenericState) in enumerate(sol.trajectory)
    println("step $i")
    for (j::Int, lm::LandmarkNode) in enumerate(rg.first.nodes)
        for f::FactPair in lm.landmark.facts
            if (rg.second.planning_graph.conditions[f.var] in s.facts) == (f.value == 1)
                if !(j in active)
                    delete!(not_reached, j)
                    push!(active, j)
                    println("    + $j")
                end
            else
                if j in active
                    delete!(active, j)
                    println("    - $j")
                end
            end
        end
    end
end
if !isempty(not_reached)
    println("Solution did not reach all landmarks!")
end
println()

cdomain, cstate = compiled(domain, state)

println("Verifying compiled")
@elapsed sol = planner(cdomain, cstate, spec)
@test is_goal(spec, cdomain, sol.trajectory[end])

# Store results in data frame
df = DataFrame(domain=String[], problem=String[], problem_size=Int[],
               compiled=Bool[], planner=String[], heuristic=String[], run=Int[],
               n_steps=Int[], time=Float64[], bytes=Int[],
               n_eval=Int[], n_expand=Int[])

# Load all problems with up to N
TIMEOUT = 180.0
NRUNS = 3
timed_out = Bool[]

df_path = "dev-test-results-$(today()).csv"

psize = get_problem_size(problem)
println("=== Problem $INSTANCE: Size $psize ===")
println()

# Repeat for both original and compiled
domname = PDDL.get_name(domain)
for domst in ((domain, state), (cdomain, cstate))
    dom, state = domst
    dom isa CompiledDomain ? println("Compiled:") : println("Interpreted:")

    # Construct heuristic and planner
    nruns = dom isa CompiledDomain ? NRUNS + 1 : NRUNS
    timed_out = false
    # Run and time planner
    for i in 1:nruns
        if timed_out continue end
        println("== run $i ==")
        stats = @timed begin
            sol = planner(dom, state, spec)
        end
        timed_out = sol.status == :max_time
        println("time: ", timed_out ? -1 : stats.time, " seconds")
        row = (
            domain = string(domname),
            problem = string(problem.name),
            problem_size = psize,
            compiled = (dom isa CompiledDomain),
            planner = string(typeof(planner)),
            heuristic = "-",
            run = i,
            time = timed_out ? -1.0 : stats.time,
            bytes = stats.bytes,
            n_steps = timed_out ? -1 : length(sol.plan),
            n_eval = length(sol.search_tree),
            n_expand = sol.expanded
        )
        push!(df, row)
        GC.gc()
    end
    println()
end
CSV.write(df_path, df)

builtin_df = df

## Analysis code ##

df
allowmissing!(df)
replace!(df.time, -1 => missing)

function summary_colname(pl, h, cmpl, runs)
    if pl == "OrderedLandmarksPlanner"
        if runs == 1:1
            return cmpl ? "Compiled-1st ($h)" : "Interpreted-1st ($h)"
        else
            return cmpl ? "Compiled ($h)" : "Interpreted ($h)"
        end
    else
        return pl
    end
end

prob_df = nothing
psize_df = nothing
pl = "OrderedLandmarksPlanner"
h = "HAdd"
cmpl = false
runs = 1:NRUNS

name = summary_colname(pl, h, cmpl, runs)
println("Summarizing data for: $name")
fdf = filter(df) do row
    (row.planner == pl && row.heuristic == h &&
        row.compiled == cmpl && row.run in runs)
end
# Summarize for each problem
p_df = combine(groupby(fdf, :problem),
                :time => (x -> mean(skipmissing(x))) => name,
                :time => (x -> std(skipmissing(x))) => "Std $name")
prob_df = prob_df === nothing ?
    p_df : outerjoin(prob_df, p_df, on=:problem)
# Summaize for each problem size
ps_df = combine(groupby(fdf, :problem_size),
                :time => (x -> mean(skipmissing(x))) => name,
                :time => (x -> std(skipmissing(x))) => "Std $name")
psize_df = psize_df === nothing ?
    ps_df : outerjoin(psize_df, ps_df, on=:problem_size)

prob_df_path = "$DOMAIN_NAME-problem-summary-$(today()).csv"
psize_df_path = "$DOMAIN_NAME-psize-summary-$(today()).csv"
CSV.write(prob_df_path, prob_df)
CSV.write(psize_df_path, psize_df)

ratio_df_path = "$DOMAIN_NAME-ratio-expand-summary-$(today()).csv"
CSV.write(ratio_df_path, ratio_df)
