# Test that planners correctly solve simple problems

@testset "Planners" begin

# Load domains and problems
path = joinpath(dirname(pathof(SymbolicPlanners)), "..", "domains", "gridworld")
gridworld = load_domain(joinpath(path, "domain.pddl"))
gw_problem = load_problem(joinpath(path, "problem-1.pddl"))
gw_state = init_state(gw_problem)
gw_spec = Specification(gw_problem)

path = joinpath(dirname(pathof(SymbolicPlanners)), "..", "domains", "doors-keys-gems")
doors_keys_gems = load_domain(joinpath(path, "domain.pddl"))
dkg_problem = load_problem(joinpath(path, "problem-1.pddl"))
dkg_state = init_state(dkg_problem)
dkg_spec = Specification(dkg_problem)

path = joinpath(dirname(pathof(SymbolicPlanners)), "..", "domains", "blocksworld")
blocksworld = load_domain(joinpath(path, "domain.pddl"))
bw_problem = load_problem(joinpath(path, "problem-0.pddl"))
bw_state = init_state(bw_problem)
bw_spec = Specification(bw_problem)

@testset "Breadth-First Planner" begin

planner = BreadthFirstPlanner()
sol = planner(gridworld, gw_state, gw_spec)
@test is_goal(gw_spec, gridworld, sol.trajectory[end])
@test collect(sol) == @pddl("down", "down", "right", "right", "up", "up")

sol = planner(doors_keys_gems, dkg_state, dkg_spec)
@test is_goal(dkg_spec, doors_keys_gems, sol.trajectory[end])
@test collect(sol) == @pddl("(down)", "(pickup key1)", "(down)",
                            "(unlock key1 right)", "(right)", "(right)",
                            "(up)", "(up)", "(pickup gem1)")

sol = planner(blocksworld, bw_state, bw_spec)
@test is_goal(bw_spec, blocksworld, sol.trajectory[end])
@test collect(sol) == @pddl("(pick-up a)", "(stack a b)",
                            "(pick-up c)", "(stack c a)")

end

@testset "Uniform Cost Planner" begin

clear_available_action_cache!()

planner = UniformCostPlanner()
sol = planner(gridworld, gw_state, gw_spec)
@test is_goal(gw_spec, gridworld, sol.trajectory[end])
@test collect(sol) == @pddl("down", "down", "right", "right", "up", "up")

planner = UniformCostPlanner()
sol = planner(doors_keys_gems, dkg_state, dkg_spec)
@test is_goal(dkg_spec, doors_keys_gems, sol.trajectory[end])
@test collect(sol) == @pddl("(down)", "(pickup key1)", "(down)",
                            "(unlock key1 right)", "(right)", "(right)",
                            "(up)", "(up)", "(pickup gem1)")

planner = UniformCostPlanner()
sol = planner(blocksworld, bw_state, bw_spec)
@test is_goal(bw_spec, blocksworld, sol.trajectory[end])
@test collect(sol) == @pddl("(pick-up a)", "(stack a b)",
                            "(pick-up c)", "(stack c a)")

end

@testset "Greedy Planner" begin

clear_available_action_cache!()

planner = GreedyPlanner(ManhattanHeuristic(@pddl("xpos", "ypos")))
sol = planner(gridworld, gw_state, gw_spec)
@test is_goal(gw_spec, gridworld, sol.trajectory[end])
@test collect(sol) == @pddl("down", "down", "right", "right", "up", "up")

planner = GreedyPlanner(GoalCountHeuristic())
sol = planner(doors_keys_gems, dkg_state, dkg_spec)
@test is_goal(dkg_spec, doors_keys_gems, sol.trajectory[end])
@test collect(sol) == @pddl("(down)", "(pickup key1)", "(down)",
                            "(unlock key1 right)", "(right)", "(right)",
                            "(up)", "(up)", "(pickup gem1)")

planner = GreedyPlanner(HAdd())
sol = planner(blocksworld, bw_state, bw_spec)
@test is_goal(bw_spec, blocksworld, sol.trajectory[end])
@test collect(sol) == @pddl("(pick-up a)", "(stack a b)",
                            "(pick-up c)", "(stack c a)")

end

@testset "A* Planner" begin

clear_available_action_cache!()

planner = AStarPlanner(ManhattanHeuristic(@pddl("xpos", "ypos")))
sol = planner(gridworld, gw_state, gw_spec)
@test is_goal(gw_spec, gridworld, sol.trajectory[end])
@test collect(sol) == @pddl("down", "down", "right", "right", "up", "up")

planner = AStarPlanner(GoalCountHeuristic())
sol = planner(doors_keys_gems, dkg_state, dkg_spec)
@test is_goal(dkg_spec, doors_keys_gems, sol.trajectory[end])
@test collect(sol) == @pddl("(down)", "(pickup key1)", "(down)",
                            "(unlock key1 right)", "(right)", "(right)",
                            "(up)", "(up)", "(pickup gem1)")

planner = AStarPlanner(HAdd())
sol = planner(blocksworld, bw_state, bw_spec)
@test is_goal(bw_spec, blocksworld, sol.trajectory[end])
@test collect(sol) == @pddl("(pick-up a)", "(stack a b)",
                            "(pick-up c)", "(stack c a)")

end

@testset "Backward Planner" begin

clear_relevant_action_cache!()

planner = BackwardPlanner(heuristic=HAddR())
sol = planner(blocksworld, bw_state, bw_spec)
spec = SymbolicPlanners.BackwardSearchGoal(bw_spec, bw_state)
@test is_goal(spec, blocksworld, sol.trajectory[1])
@test collect(sol) == @pddl("(pick-up a)", "(stack a b)",
                            "(pick-up c)", "(stack c a)")

end

@testset "Real Time Dynamic Programming" begin

clear_available_action_cache!()
Random.seed!(0)
simulator = StateActionRecorder(100)

heuristic = ManhattanHeuristic(@pddl("xpos", "ypos"))
planner = RTDP(heuristic=heuristic, rollout_noise=1.0, n_rollouts=10)
sol = planner(gridworld, gw_state, gw_spec)
actions, trajectory = simulator(sol, gridworld, gw_state, gw_spec)
@test is_goal(gw_spec, gridworld, trajectory[end])
@test actions == @pddl("down", "down", "right", "right", "up", "up")

heuristic = GoalCountHeuristic()
planner = RTDP(heuristic=heuristic, rollout_noise=1.0, n_rollouts=10)
sol = planner(doors_keys_gems, dkg_state, dkg_spec)
actions, trajectory = simulator(sol, doors_keys_gems, dkg_state, dkg_spec)
@test is_goal(dkg_spec, doors_keys_gems, trajectory[end])
@test actions == @pddl("(down)", "(pickup key1)", "(down)",
                       "(unlock key1 right)", "(right)", "(right)",
                        "(up)", "(up)", "(pickup gem1)")

planner = RTDP(heuristic=HAdd(), rollout_noise=1.0, n_rollouts=10)
sol = planner(blocksworld, bw_state, bw_spec)
actions, trajectory = simulator(sol, blocksworld, bw_state, bw_spec)
@test is_goal(bw_spec, blocksworld, trajectory[end])
@test actions == @pddl("(pick-up a)", "(stack a b)",
                       "(pick-up c)", "(stack c a)")

end

@testset "Monte Carlo Tree Search" begin

clear_available_action_cache!()
Random.seed!(0)
simulator = StateActionRecorder(100)

heuristic = ManhattanHeuristic(@pddl("xpos", "ypos"))
planner = MCTS(heuristic=heuristic, n_rollouts=50)
sol = planner(gridworld, gw_state, gw_problem.goal)
actions, trajectory = simulator(sol, gridworld, gw_state, gw_spec)
@test is_goal(gw_spec, gridworld, trajectory[end])
@test actions == @pddl("down", "down", "right", "right", "up", "up")

heuristic = GoalCountHeuristic()
planner = MCTS(heuristic=heuristic, n_rollouts=50)
sol = planner(doors_keys_gems, dkg_state, dkg_problem.goal)
actions, trajectory = simulator(sol, doors_keys_gems, dkg_state, dkg_spec)
@test is_goal(dkg_spec, doors_keys_gems, trajectory[end])
@test actions == @pddl("(down)", "(pickup key1)", "(down)",
                       "(unlock key1 right)", "(right)", "(right)",
                        "(up)", "(up)", "(pickup gem1)")

planner = MCTS(heuristic=HAdd(), n_rollouts=50)
sol = planner(blocksworld, bw_state, bw_problem.goal)
actions, trajectory = simulator(sol, blocksworld, bw_state, bw_spec)
@test is_goal(bw_spec, blocksworld, trajectory[end])
@test actions == @pddl("(pick-up a)", "(stack a b)",
                       "(pick-up c)", "(stack c a)")

end

end
