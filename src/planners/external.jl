export FastDownward, Pyperplan
export ExternalPlan

"Solution type for plans produced by external planners."
struct ExternalPlan <: OrderedSolution
    plan::Vector{Term}
    runtime::Float64
    expanded::Int
end

ExternalPlan(plan::AbstractVector{<:Term}) = ExternalPlan(plan, -1, -1)

get_action(sol::ExternalPlan, t::Int) = sol.plan[t]

Base.iterate(sol::ExternalPlan) = iterate(sol.plan)
Base.iterate(sol::ExternalPlan, istate) = iterate(sol.plan, istate)
Base.getindex(sol::ExternalPlan, i::Int) = getindex(sol.plan, i)
Base.length(sol::ExternalPlan) = length(sol.plan)

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

"Calls the FastDownward planning system to produce a plan."
function solve(planner::FastDownward,
               domain::Domain, state::State, spec::Specification)
    if isempty(planner.fd_path)
        error("fd_path not set to location of fast_downward.py")
    end
    @unpack search, heuristic, h_params, max_time, verbose = planner
    @unpack fd_path, py_cmd = planner
    # Write temporary domain and problem files
    goal = Compound(:and, get_goal_terms(spec))
    metric = hasfield(typeof(spec), :metric) ? spec.metric : nothing
    if metric != nothing metric = (-1, metric) end
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
        return ExternalPlan(plan, runtime, expanded)
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

"Calls Pyperplan to produce a plan."
function solve(planner::Pyperplan,
               domain::Domain, state::State, spec::Specification)
    @unpack search, heuristic, log_level, max_time, verbose, py_cmd = planner
    # Write temporary domain and problem files
    goal = Compound(:and, get_goal_terms(spec))
    metric = hasfield(typeof(spec), :metric) ? spec.metric : nothing
    if metric != nothing metric = (-1, metric) end
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
    plan = readlines(sol_path)[1:end-1]
    Base.Filesystem.rm(sol_path)
    plan = parse_pddl.(plan)
    # Return plan with statistics if flag is set
    if planner.log_stats
        m = match(r"(\d+) Nodes expanded", output)
        expanded = m === nothing ? -1 : parse(Int, m.captures[1])
        return ExternalPlan(plan, runtime, expanded)
    end
    return ExternalPlan(plan)
end
