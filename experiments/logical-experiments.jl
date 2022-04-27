## Benchmarking experiments for planners on propositional domains ##
using PDDL, SymbolicPlanners, Test
using DataFrames, CSV, Dates, Statistics

DOMAIN_NAME = "blocksworld" # One of "blocksworld", "logistics", "miconic"
PROBLEM_SIZE_REGEX = Dict(
    "blocksworld" => r"\w+-(\d+)-(\d+)",
    "logistics" => r"\w+-(\d+)-(\d+)",
    "miconic" => r"\w+-f\d+-p(\d+)-.*"
)
function get_problem_size(problem)
    m = match(PROBLEM_SIZE_REGEX[DOMAIN_NAME], string(problem.name))
    return parse(Int, m.captures[1])
end

## Load domain and problem for initial test runs ##

domain_dir = joinpath(@__DIR__, "logical", DOMAIN_NAME)
domain = load_domain(joinpath(domain_dir, "domain.pddl"))
problem = load_problem(joinpath(domain_dir, "instance-19.pddl"))

state = initstate(domain, problem)
spec = Specification(problem)

planner = AStarPlanner(HAdd(), save_search=true)
@elapsed sol = planner(domain, state, spec)
@test is_goal(spec, domain, sol.trajectory[end])

cdomain, cstate = compiled(domain, state)

@elapsed sol = planner(cdomain, cstate, spec)
@test is_goal(spec, cdomain, sol.trajectory[end])

## Run experiments for built-in A* planner ##

# Store results in data frame
df = DataFrame(domain=String[], problem=String[], problem_size=Int[],
               compiled=Bool[], planner=String[], heuristic=String[], run=Int[],
               n_steps=Int[], time=Float64[], bytes=Int[],
               n_eval=Int[], n_expand=Int[])
heuristics = Dict("GoalCount" => GoalCountHeuristic())

# Load all problems with up to N
N = 26
TIMEOUT = 180.0
NRUNS = 3
problem_set = [joinpath(domain_dir, "instance-$i.pddl") for i in 1:N];
timed_out = Bool[]

df_path = "$DOMAIN_NAME-results-$(today()).csv"

for (problem_id, problem_path) in enumerate(problem_set)
    problem = load_problem(problem_path)
    psize = get_problem_size(problem)
    println("=== Problem $problem_id: Size $psize ===")
    println()
    # Initialize state and specification
    domname = PDDL.get_name(domain)
    state = initstate(domain, problem)
    spec = Specification(problem)
    # Compile domain
    cdomain, cstate = compiled(domain, state)
    # Repeat for both original and compiled
    for dom in (domain, cdomain), hname in keys(heuristics)
        if !(dom isa CompiledDomain) continue end
        dom isa CompiledDomain ?
            println("Compiled ($hname):") : println("Interpreted ($hname):")
        state = initstate(dom, problem)
        # Construct heuristic and planner
        h = deepcopy(heuristics[hname])
        planner = AStarPlanner(h, max_time=TIMEOUT, save_search=true)
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
                heuristic = hname,
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
end

builtin_df = df

## Test runs for external planners ##

# ENV["PYTHON"] = "python"
# ENV["FD_PATH"] = "../FastDownward/fast-downward.py"

planner = Pyperplan(search="astar", heuristic="hadd", verbose=true)
sol = planner(domain, state, spec)

planner = FastDownward(search="astar", heuristic="add", verbose=true)
sol = planner(domain, state, spec)

## Run experiments for external planners ##

df = DataFrame(domain=String[], problem=String[], problem_size=Int[],
               compiled=Bool[], planner=String[], heuristic=String[], run=Int[],
               n_steps=Int[], time=Float64[], bytes=Int[],
               n_eval=Int[], n_expand=Int[])

external_planners = [
#     Pyperplan(search="astar", heuristic="hadd"),
    FastDownward(search="astar", heuristic="add")
]

df_path = "$DOMAIN_NAME-results-external-$(today()).csv"

for (problem_id, problem_path) in enumerate(problem_set)
    problem = load_problem(problem_path)
    psize = get_problem_size(problem)
    println("=== Problem $problem_id: Size $psize ===")
    println()
    # Initialize state and specification
    domname = PDDL.get_name(domain)
    state = initstate(domain, problem)
    spec = Specification(problem)
    for planner in external_planners
        state = initstate(domain, problem)
        # Construct heuristic and planner
        nruns = NRUNS
        # Run and time planner
        for i in 1:nruns
            println("== run $i ==")
            sol = planner(domain, state, spec)
            runtime = sol isa NullSolution ? -1 : sol.runtime
            println("time: ", runtime, " seconds")
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
    ("ForwardPlanner", "GoalCount", false, 1:NRUNS),
    ("ForwardPlanner", "HAdd", false, 1:NRUNS),
    ("ForwardPlanner", "GoalCount", true, 1:1),
    ("ForwardPlanner", "HAdd", true, 1:1),
    ("ForwardPlanner", "GoalCount", true, 2:NRUNS+1),
    ("ForwardPlanner", "HAdd", true, 2:NRUNS+1),
    ("Pyperplan", "HAdd", false, 1:NRUNS),
    ("FastDownward", "HAdd", false, 1:NRUNS),
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

ratio_df_path = "$DOMAIN_NAME-ratio-expand-summary-$(today()).csv"
CSV.write(ratio_df_path, ratio_df)
