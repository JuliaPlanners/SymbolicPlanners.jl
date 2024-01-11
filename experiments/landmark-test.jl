# Run using 'julia --project=. experiments/landmark-test.jl'
# Can also be ran using 'julia ./experiments/landmark-test.jl' after adding all the bellow packages in the julia pkg manager
using PDDL, SymbolicPlanners, Test, PlanningDomains

println("Started")
# Load Blocksworld domain and single problem
domain = load_domain(:blocksworld)
problem = load_problem(:blocksworld, "problem-2")

# Initialize state
state = initstate(domain, problem)
spec = Specification(problem)

#Create LM graph
lm_graph::LandmarkGraph, gen_data::SymbolicPlanners.LandmarkGenerationData = compute_relaxed_landmark_graph(domain, state, spec)
approximate_reasonable_orders(lm_graph, gen_data)
landmark_graph_remove_cycles_complete(lm_graph)
landmark_graph_draw_png("graph.png", lm_graph, gen_data.planning_graph)

# Add our planner here
# planner = LMLocalPlanner(lm_graph, gen_data.planning_graph, AStarPlanner(HAdd(), save_search=true), 180.0)
planner = AStarPlanner(LMCount(lm_graph, gen_data.planning_graph), max_time=180.0, save_search=true)
# planner = LMLocalSmartPlanner(lm_graph, gen_data, AStarPlanner(HAdd(), save_search=true), 180.0)
## Run Planner ##

println("Verifying interpreted")
stats = @timed begin
    sol = planner(domain, state, spec)
end
println(spec)
println(sol.trajectory[end])
@test is_goal(spec, domain, sol.trajectory[end])
println("Interpreted Finished in ", stats.time, " seconds")
println("Eval: $(length(sol.search_tree)) - Expand: $(sol.expanded)")

cdomain, cstate = compiled(domain, state)

println("Verifying compiled")
stats = @timed begin
    sol = planner(cdomain, cstate, spec)
end

@test is_goal(spec, cdomain, sol.trajectory[end])
println("Compiled Finished ", stats.time, " seconds")
