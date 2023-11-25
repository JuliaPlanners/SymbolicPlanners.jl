## Benchmarking experiments for planners on numeric domains ##
using PDDL, SymbolicPlanners, Test
using DataFrames, CSV, Dates, Statistics

NRUNS = 3
DOMAIN_NAME = "zeno-travel" # One of "zeno-travel", "depots", "rovers"
SATISFICING = true
function get_problem_size(problem)
    return length(problem.objects)
end

## Load domain and problem for initial test runs ##

domain_dir = joinpath(@__DIR__, "numeric", DOMAIN_NAME)
domain = load_domain(joinpath(domain_dir, "domain.pddl"))
problem = load_problem(joinpath(domain_dir, "instance-1.pddl"))

state = initstate(domain, problem)
state[pddl"total-time"] = 0.0
spec = SATISFICING ? MinStepsGoal(problem) : MinActionCosts(domain, problem);

planner = AStarPlanner(HAdd(), save_search=true, max_time=180)
@elapsed sol = planner(domain, state, spec)
@test is_goal(spec, domain, sol.trajectory[end])

cdomain, cstate = compiled(domain, state)

@elapsed sol = planner(cdomain, cstate, spec)
@test is_goal(spec, cdomain, sol.trajectory[end])

## Run experiments for built-in A* planner ##

# Store results in data frame
df = DataFrame(domain=String[], problem=String[], problem_size=Int[],
               compiled=Bool[], planner=String[], heuristic=String[], run=Int[],
               n_steps=Int[], metric=Float64[], time=Float64[], bytes=Int[],
               n_eval=Int[], n_expand=Int[])
heuristics = Dict("HAdd" => HAdd())

# Load all problems with up to 10 blocks
N = 10
TIMEOUT = 180.0
problem_set = [joinpath(domain_dir, "instance-$i.pddl") for i in 1:N]

sat_str = SATISFICING ? "sat" : "opt"
df_path = "$DOMAIN_NAME-results-$sat_str-$(today()).csv"

for (problem_id, problem_path) in enumerate(problem_set)
    problem = load_problem(problem_path)
    psize = get_problem_size(problem)
    println("=== Problem $problem_id: Size $psize ===")
    println()
    # Initialize state and specification
    domname = PDDL.get_name(domain)
    state = initstate(domain, problem)
    # state[pddl"total-time"] = 0.0
    spec = SATISFICING ? MinStepsGoal(problem) : MinMetricGoal(problem)
    metric = PDDL.get_metric(problem).args[1]
    # Compile domain
    cdomain, cstate = compiled(domain, state)
    # Repeat for both original and compiled
    for dom in (domain, cdomain), hname in keys(heuristics)
        dom isa CompiledDomain ?
            println("Compiled ($hname):") : println("Interpreted ($hname):")
        state = initstate(dom, problem)
        if DOMAIN_NAME == "zeno-travel"
            state[pddl"total-time"] = 0.0
        end
        # Construct heuristic and planner
        h = deepcopy(heuristics[hname])
        planner = ForwardPlanner(heuristic=h, max_time=TIMEOUT, save_search=true)
        nruns = dom isa CompiledDomain ? NRUNS + 1 : NRUNS
        timeout = false
        # Run and time planner
        for i in 1:nruns
            if timeout continue end
            println("== run $i ==")
            stats = @timed begin
                sol = planner(dom, state, spec)
            end
            println("time: ", stats.time, " seconds")
            timeout = sol.status == :max_time
            metric_val = (sol.status == :success && metric != pddl"total-time") ?
                evaluate(dom, sol.trajectory[end], metric) : -1.0
            row = (
                domain = string(domname),
                problem = string(problem.name),
                problem_size = psize,
                compiled = (dom isa CompiledDomain),
                planner = string(typeof(planner)),
                heuristic = hname,
                run = i,
                time = sol.status == :max_time ? -1.0 : stats.time,
                bytes = stats.bytes,
                n_steps = sol.status == :max_time ? -1 : length(sol.plan),
                metric = metric_val,
                n_eval = length(sol.search_tree),
                n_expand = sol.expanded
            )
            push!(df, row)
            GC.gc()
        end
        println()
    end
    CSV.write(df_path, df)
end

builtin_df = df

## Test runs for external planners ##

# ENV["JAVA"] = "java"
# ENV["ENHSP_PATH"] = "../enhsp-dist/enhsp.jar"

spec = SATISFICING ? MinStepsGoal(problem) : MinMetricGoal(problem)
state = initstate(domain, problem)

planner = ENHSP(search="gbfs", heuristic="hadd", verbose=true)
sol = planner(domain, state, spec)

## Run experiments for external planners ##

df = DataFrame(domain=String[], problem=String[], problem_size=Int[],
               compiled=Bool[], planner=String[], heuristic=String[], run=Int[],
               n_steps=Int[], metric=Float64[], time=Float64[], bytes=Int[],
               n_eval=Int[], n_expand=Int[])

search_str = SATISFICING ? "WAStar" : "WAStar"
external_planners = [
    ENHSP(search=search_str, heuristic="hadd", verbose=false)
]

sat_str = SATISFICING ? "sat" : "opt"
df_path = "$DOMAIN_NAME-results-$sat_str-external-$(today()).csv"

for (problem_id, problem_path) in enumerate(problem_set)
    problem = load_problem(problem_path)
    psize = get_problem_size(problem)
    println("=== Problem $problem_id: Size $psize ===")
    println()
    # Initialize state and specification
    domname = PDDL.get_name(domain)
    state = initstate(domain, problem)
    if DOMAIN_NAME == "zeno-travel"
        state[pddl"total-time"] = 0.0
    end
    spec = SATISFICING ? MinStepsGoal(problem) : MinMetricGoal(problem)
    metric_spec = MinMetricGoal(problem)
    for planner in external_planners
        # Construct heuristic and planner
        nruns = NRUNS
        timeout = false
        # Run and time planner
        for i in 1:nruns
            if timeout continue end
            println("== run $i ==")
            sol = planner(domain, state, spec)
            runtime = sol isa NullSolution ? -1 : sol.runtime
            timeout = sol isa NullSolution && sol.status == :max_time
            println("time: ", runtime, " seconds")
            metric_val =  sol isa NullSolution ?
                -1.0 : evaluate(domain, state, metric_spec.metric)
            row = (
                domain = string(domname),
                problem = string(problem.name),
                problem_size = psize,
                compiled = false,
                planner = string(typeof(planner)),
                heuristic = "HAdd",
                run = i,
                time = runtime,
                bytes = -1,
                n_steps = sol isa NullSolution ? -1 : length(sol.plan),
                metric = metric_val,
                n_eval = sol isa NullSolution ? -1 : sol.evaluated,
                n_expand = sol isa NullSolution ? -1 : sol.expanded
            )
            push!(df, row)
        end
        println()
    end
    CSV.write(df_path, df)
end

external_df = df

## Analysis code ##

NRUNS = 3

df = vcat(builtin_df, external_df)
allowmissing!(df)
replace!(df.time, -1 => missing)

function summary_colname(pl, h, cmpl, runs)
    if pl == "ForwardPlanner"
        if runs == 1:1
            return cmpl ? "Compiled-1st ($h)" : "Interpreted-1st ($h)"
        else
            return cmpl ? "Compiled ($h)" : "Interpreted ($h)"
        end
    else
        return pl
    end
end

selections = [
    ("ForwardPlanner", "HAdd", false, 1:NRUNS),
    ("ForwardPlanner", "HAdd", true, 1:1),
    ("ForwardPlanner", "HAdd", true, 2:NRUNS+1),
    ("ENHSP", "HAdd", false, 1:NRUNS),
]

# Summarize data for each problem and problem size
prob_df = nothing
psize_df = nothing
for (pl, h, cmpl, runs) in selections
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
end

prob_df_path = "$DOMAIN_NAME-problem-summary-$(today()).csv"
psize_df_path = "$DOMAIN_NAME-psize-summary-$(today()).csv"
CSV.write(prob_df_path, prob_df)
CSV.write(psize_df_path, psize_df)

# Compute runtime ratios
df = select(prob_df, [n for n in names(prob_df) if n[1:3] != "Std"])
allowmissing!(df)
for col in eachcol(df)
    replace!(col, NaN => missing)
end
df[!, Not(:problem)] ./= df[!, "Compiled (HAdd)"]
ratio_df = describe(df[!, Not(:problem)],
                    :q25, :median, :q75, :mean, :std, :nmissing)

ratio_df_path = "$DOMAIN_NAME-ratio-summary-$(today()).csv"
CSV.write(ratio_df_path, ratio_df)
