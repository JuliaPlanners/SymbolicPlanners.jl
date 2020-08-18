# Test that planners correctly solve simple problems

@testset "Planners" begin

# Load domains and problems
path = joinpath(dirname(pathof(SymbolicPlanners)), "..", "domains", "gridworld")
gridworld = load_domain(joinpath(path, "domain.pddl"))
gw_problem = load_problem(joinpath(path, "problem-1.pddl"))
gw_state = init_state(gw_problem)

path = joinpath(dirname(pathof(SymbolicPlanners)), "..", "domains", "doors-keys-gems")
doors_keys_gems = load_domain(joinpath(path, "domain.pddl"))
dkg_problem = load_problem(joinpath(path, "problem-1.pddl"))
dkg_state = init_state(dkg_problem)

path = joinpath(dirname(pathof(SymbolicPlanners)), "..", "domains", "blocksworld")
blocksworld = load_domain(joinpath(path, "domain.pddl"))
bw_problem = load_problem(joinpath(path, "problem-0.pddl"))
bw_state = init_state(bw_problem)

@testset "BFS Planner" begin

planner = BFSPlanner()
plan, traj = planner(gridworld, gw_state, gw_problem.goal)
@test satisfy(gw_problem.goal, traj[end], gridworld)[1] == true
@test plan == @julog [down, down, right, right, up, up]

plan, traj = planner(doors_keys_gems, dkg_state, dkg_problem.goal)
@test satisfy(dkg_problem.goal, traj[end], doors_keys_gems)[1] == true
@test plan == @julog [down, pickup(key1), down, unlock(key1, right),
                      right, right, up, up, pickup(gem1)]

plan, traj = planner(blocksworld, bw_state, bw_problem.goal)
@test satisfy(bw_problem.goal, traj[end], blocksworld)[1] == true
@test plan == [pddl"(pick-up a)", pddl"(stack a b)",
               pddl"(pick-up c)", pddl"(stack c a)"]

end

@testset "A* Planner" begin

clear_heuristic_cache!()
clear_available_action_cache!()

planner = AStarPlanner(heuristic=ManhattanHeuristic(@julog[xpos, ypos]))
plan, traj = planner(gridworld, gw_state, gw_problem.goal)
@test satisfy(gw_problem.goal, traj[end], gridworld)[1] == true
@test plan == @julog [down, down, right, right, up, up]

planner = AStarPlanner(heuristic=GoalCountHeuristic())
plan, traj = planner(doors_keys_gems, dkg_state, dkg_problem.goal)
@test satisfy(dkg_problem.goal, traj[end], doors_keys_gems)[1] == true
@test plan == @julog [down, pickup(key1), down, unlock(key1, right),
                      right, right, up, up, pickup(gem1)]

planner = AStarPlanner(heuristic=HAdd())
plan, traj = planner(blocksworld, bw_state, bw_problem.goal)
@test satisfy(bw_problem.goal, traj[end], blocksworld)[1] == true
@test plan == [pddl"(pick-up a)", pddl"(stack a b)",
               pddl"(pick-up c)", pddl"(stack c a)"]

end

@testset "Backward A* Planner" begin

clear_heuristic_cache!()
clear_relevant_action_cache!()

planner = BackwardAStarPlanner(heuristic=HAddR())
plan, traj = planner(blocksworld, bw_state, bw_problem.goal)
@test issubset(traj[1], bw_state) == true
@test plan == [pddl"(pick-up a)", pddl"(stack a b)",
               pddl"(pick-up c)", pddl"(stack c a)"]

end

end
