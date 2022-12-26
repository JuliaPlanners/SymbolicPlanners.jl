export FastDownward, Pyperplan, ENHSP
export ExternalPlan

"Solution type for plans produced by external planners."
@auto_hash_equals struct ExternalPlan <: OrderedSolution
    status::Symbol
    plan::Vector{Term}
    runtime::Float64
    expanded::Int
    evaluated::Int
end

ExternalPlan(plan::AbstractVector{<:Term}) =
    ExternalPlan(:success, plan, -1, -1, -1)

ExternalPlan(plan::AbstractVector{<:Term}, runtime, expanded, evaluated) =
    ExternalPlan(:success, plan, runtime, expanded, evaluated)

Base.copy(sol::ExternalPlan) = 
    ExternalPlan(sol.status, copy(sol.plan),
                 sol.runtime, sol.expanded, sol.evaluated)

get_action(sol::ExternalPlan, t::Int) = sol.plan[t]

Base.iterate(sol::ExternalPlan) = iterate(sol.plan)
Base.iterate(sol::ExternalPlan, istate) = iterate(sol.plan, istate)
Base.getindex(sol::ExternalPlan, i::Int) = getindex(sol.plan, i)
Base.length(sol::ExternalPlan) = length(sol.plan)

"Get metric expression from metric."
get_metric_expr(spec::Specification) = nothing
get_metric_expr(spec::MinMetricGoal) = Compound(:minimize, [spec.metric])
get_metric_expr(spec::MaxMetricGoal) = Compound(:maximize, [spec.metric])

"Wrapper to the FastDownward planning system."
@kwdef mutable struct FastDownward <: Planner
    search::String = "astar" # Search algorithm
    heuristic::String = "add" # Search heuristic
    h_params::Dict{String, String} = Dict() # Heuristic parameters
    max_time::Float64 = 300 # Time limit
    verbose::Bool = false # Whether to print planner outputs
    log_stats::Bool = true # Whether to log statistics
    fd_path::String = get(ENV, "FD_PATH", "") # Path to fast_downward.py
    py_cmd::String = get(ENV, "PYTHON", "python") # Python path
end

@auto_hash FastDownward
@auto_equals FastDownward

function Base.copy(p::FastDownward)
    return FastDownward(p.search, p.heuristic, p.h_params, p.max_time,
                        p.max_time, p.verbose, p.log_stats, p.fd_path, p.py_cmd)
end

function solve(planner::FastDownward,
               domain::Domain, state::State, spec::Specification)
    if isempty(planner.fd_path)
        error("fd_path not set to location of fast_downward.py")
    end
    @unpack search, heuristic, h_params, max_time, verbose = planner
    @unpack fd_path, py_cmd = planner
    # Write temporary domain and problem files
    goal = Compound(:and, get_goal_terms(spec))
    metric = get_metric_expr(spec)
    problem = GenericProblem(state, goal=goal, metric=metric,
                             domain=PDDL.get_name(domain))
    domain_path = save_domain(tempname(), domain)
    problem_path = save_problem(tempname(), problem)
    # Set up shell command to fast_downward.py
    h_params = join(["$key=$val" for (key, val) in h_params], ", ")
    search_params = "$search($heuristic($h_params))"
    cmd = `$py_cmd $fd_path $domain_path $problem_path --search $search_params`
    # Run command up to max time
    out = Pipe()
    proc = run(pipeline(cmd, stdout=out); wait=false)
    cb() = process_exited(proc)
    timedwait(cb, float(max_time))
    if process_running(proc)
        @debug "Planner timed out."
        Base.Filesystem.rm(domain_path); Base.Filesystem.rm(problem_path)
        kill(proc); close(out.in)
        return NullSolution(:max_time)
    end
    # Read output and check if solution was found
    close(out.in)
    output = read(out, String)
    Base.Filesystem.rm(domain_path); Base.Filesystem.rm(problem_path)
    if verbose print(output) end
    if !occursin("Solution found", output)
        if occursin("aborting after translate", output)
            error("Could not translate domain and problem files.")
        end
        return NullSolution(:failure)
    end
    # Read plan from file
    plan = readlines("./sas_plan")[1:end-1]
    Base.Filesystem.rm("./sas_plan")
    plan = parse_pddl.(plan)
    # Return plan with statistics if flag is set
    if planner.log_stats
        m = match(r"Total time: ([\d.]+)s", output)
        runtime = m === nothing ? -1 : parse(Float64, m.captures[1])
        m = match(r"Expanded (\d+) state\(s\)", output)
        expanded = m === nothing ? -1 : parse(Int, m.captures[1])
        m = match(r"Evaluated (\d+) state\(s\)", output)
        evaluated = m === nothing ? -1 : parse(Int, m.captures[1])
        return ExternalPlan(plan, runtime, expanded, evaluated)
    end
    return ExternalPlan(plan)
end

"Wrapper to the Pyperplan light-weight planner."
@kwdef mutable struct Pyperplan <: Planner
    search::String = "astar" # Search algorithm
    heuristic::String = "hadd" # Search heuristic
    log_level::String = "info" # Log level
    log_stats::Bool = true # Whether to log statistics
    max_time::Float64 = 300 # Time limit
    verbose::Bool = false # Whether to print planner outputs
    py_cmd::String = get(ENV, "PYTHON", "python") # Python path
end

@auto_hash Pyperplan
@auto_equals Pyperplan

function Base.copy(p::Pyperplan)
    return Pyperplan(p.search, p.heuristic, p.log_level, p.log_stats,
                     p.max_time, p.verbose, p.py_cmd)
end

function solve(planner::Pyperplan,
               domain::Domain, state::State, spec::Specification)
    @unpack search, heuristic, log_level, max_time, verbose, py_cmd = planner
    # Write temporary domain and problem files
    goal = Compound(:and, get_goal_terms(spec))
    metric = get_metric_expr(spec)
    problem = GenericProblem(state, goal=goal, metric=metric,
                             domain=PDDL.get_name(domain))
    domain_path = save_domain(tempname(), domain)
    problem_path = save_problem(tempname(), problem)
    # Set up shell command to call pyperplan
    cmd = ```$py_cmd -m pyperplan -l $log_level -H $heuristic -s $search
             $domain_path $problem_path```
    # Run command up to max time
    start_time = time()
    out = Pipe()
    proc = run(pipeline(cmd, stdout=out); wait=false)
    cb() = process_exited(proc)
    timedwait(cb, float(max_time))
    if process_running(proc)
        @debug "Planner timed out."
        Base.Filesystem.rm(domain_path); Base.Filesystem.rm(problem_path)
        kill(proc); close(out.in)
        return NullSolution(:max_time)
    end
    runtime = time() - start_time
    # Read output and print if verbose flag is true
    close(out.in)
    output = read(out, String)
    Base.Filesystem.rm(domain_path); Base.Filesystem.rm(problem_path)
    if verbose print(output) end
    # Check if solution is output
    sol_path = splitext(problem_path)[1] * ".soln"
    if !isfile(sol_path)
        if verbose println("Solution not found.") end
        return NullSolution(:failure)
    end
    # Read plan from file
    plan = readlines(sol_path)
    Base.Filesystem.rm(sol_path)
    plan = parse_pddl.(plan)
    # Return plan with statistics if flag is set
    if planner.log_stats
        overhead = @elapsed run(pipeline(`$py_cmd -m pyperplan -h`, devnull))
        runtime -= overhead
        m = match(r"(\d+) Nodes expanded", output)
        expanded = m === nothing ? -1 : parse(Int, m.captures[1])
        return ExternalPlan(plan, runtime, expanded, -1)
    end
    return ExternalPlan(plan)
end

"Wrapper to the ENHSP numeric planner."
@kwdef mutable struct ENHSP <: Planner
    search::String = "gbfs" # Search algorithm
    heuristic::String = "hadd" # Search heuristic
    h_mult::Float64 = 1.0 # Heuristic multiplier for weighted A*
    log_stats::Bool = true # Whether to log statistics
    max_time::Float64 = 300 # Time limit
    verbose::Bool = false # Whether to print planner outputs
    enhsp_path::String = get(ENV, "ENHSP_PATH", "") # Path to enhsp.jar
    java_cmd::String = get(ENV, "JAVA", "java") # Java path
end

@auto_hash ENHSP
@auto_equals ENHSP

function Base.copy(p::ENHSP)
    return ENHSP(p.search, p.heuristic, p.h_mult, p.log_stats,
                 p.max_time, p.verbose, p.enhsp_path, p.java_cmd)
end

function solve(planner::ENHSP,
               domain::Domain, state::State, spec::Specification)
    @unpack search, heuristic, h_mult = planner
    @unpack max_time, verbose, enhsp_path, java_cmd = planner
    # Write temporary domain and problem files
    goal = Compound(:and, get_goal_terms(spec))
    metric = get_metric_expr(spec)
    problem = GenericProblem(state, goal=goal, metric=metric,
                             domain=PDDL.get_name(domain))
    domain_path = save_domain(tempname() * ".pddl", domain)
    problem_path = save_problem(tempname() * ".pddl", problem)
    sol_path = tempname() * ".pddl"
    # Set up shell command to call pyperplan
    cmd = ```$java_cmd -jar $enhsp_path -h $heuristic -s $search
             --domain $domain_path --problem $problem_path -sp $sol_path```
    # Run command up to max time
    out = Pipe()
    proc = run(pipeline(cmd, stdout=out); wait=false)
    cb() = process_exited(proc)
    timedwait(cb, float(max_time))
    if process_running(proc)
        @debug "Planner timed out."
        Base.Filesystem.rm(domain_path); Base.Filesystem.rm(problem_path)
        kill(proc); close(out.in)
        return NullSolution(:max_time)
    end
    # Read output and print if verbose flag is true
    close(out.in)
    output = read(out, String)
    Base.Filesystem.rm(domain_path); Base.Filesystem.rm(problem_path)
    if verbose print(output) end
    # Check if solution is output
    if !occursin("Problem Solved", output) || !isfile(sol_path)
        return NullSolution(:failure)
    end
    # Read plan from file
    plan = readlines(sol_path)
    Base.Filesystem.rm(sol_path)
    plan = parse_pddl.(plan)
    # Return plan with statistics if flag is set
    if planner.log_stats
        m = match(r"Expanded Nodes.*:\s*(\d+)", output)
        expanded = m === nothing ? -1 : parse(Int, m.captures[1])
        m = match(r"States Evaluated.*:\s*(\d+)", output)
        evaluated = m === nothing ? -1 : parse(Int, m.captures[1])
        m = match(r"Planning Time.*:\s*(\d+)", output)
        runtime = m === nothing ? NaN : parse(Float64, m.captures[1]) / 1000
        return ExternalPlan(plan, runtime, expanded, evaluated)
    end
    return ExternalPlan(plan)
end
