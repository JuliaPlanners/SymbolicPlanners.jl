export FastDownwardPlanner

"Wrapper to the FastDownward planning system."
@kwdef mutable struct FastDownwardPlanner <: Planner
    search::String = "astar"
    heuristic::String = "add"
    h_params::Dict{String, String} = Dict()
    timeout::Float64 = 10
    verbose::Bool = false
end

set_max_resource(planner::FastDownwardPlanner, val) =
    @set planner.timeout = val

"Calls the FastDownward planning system to produce a plan."
function call(planner::FastDownwardPlanner,
              domain::Domain, state::State, goal_spec::GoalSpec)
    if !haskey(ENV, "FD_PATH")
        error("FD_PATH not set to location of fast_downward.py") end
    @unpack search, heuristic, h_params, timeout, verbose = planner
    # Write temporary domain and problem files
    @unpack goals, metric = goal_spec
    if metric != nothing metric = (-1, metric) end
    problem = Problem(state, Compound(:and, goals), metric; domain=domain.name)
    domain_path = save_domain(tempname(), domain)
    problem_path = save_problem(tempname(), problem)
    # Set up shell command to fast_downward.py
    fd_path = ENV["FD_PATH"]
    py_cmd = get(ENV, "PYTHON", "python")
    h_params = join(["$key=$val" for (key, val) in h_params], ", ")
    search_params = "$search($heuristic($h_params))"
    cmd = `$py_cmd $fd_path $domain_path $problem_path --search $search_params`
    # Run command up to timeout
    out = Pipe()
    proc = run(pipeline(cmd, stdout=out); wait=false)
    cb() = process_exited(proc)
    timedwait(cb, float(timeout))
    if process_running(proc)
        @debug "Planner timed out."
        Base.Filesystem.rm(domain_path); Base.Filesystem.rm(problem_path)
        kill(proc); close(out.in); return nothing, nothing end
    # Read output and check if solution was found
    close(out.in)
    output = read(out, String)
    Base.Filesystem.rm(domain_path); Base.Filesystem.rm(problem_path)
    if verbose println(output) end
    if !occursin("Solution found", output)
        if occursin("aborting after translate", output)
            error("Could not translate domain and problem files.") end
        return nothing, nothing end
    # Read plan from file
    plan = readlines("./sas_plan")[1:end-1]
    Base.Filesystem.rm("./sas_plan")
    plan = parse_pddl.(plan)
    return BasicSolution(plan)
end
