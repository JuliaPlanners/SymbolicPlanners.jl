@testset "Simulators" begin

# Set up Blocksworld plan and (optimal) policy to simulate
bw_plan = OrderedPlan(@pddl("(pick-up a)", "(stack a b)",
                            "(pick-up c)", "(stack c a)"))
bw_policy = FunctionalVPolicy(PlannerHeuristic(AStarPlanner(HMax())),
                              blocksworld, bw_spec)

# Manually compute Blocksworld trajectory
bw_traj = [bw_state]
s = bw_state
for act in bw_plan
    s = transition(blocksworld, s, act)
    push!(bw_traj, s)
end

@testset "End State Simulator" begin

sim = EndStateSimulator()

end_state = sim(bw_plan, blocksworld, bw_state, bw_spec)
@test end_state == bw_traj[end]

end_state = sim(bw_policy, blocksworld, bw_state, bw_spec)
@test end_state == bw_traj[end]

end

@testset "State Recorder" begin

sim = StateRecorder()

traj = sim(bw_plan, blocksworld, bw_state, bw_spec)
@test traj == bw_traj

end_state = sim(bw_policy, blocksworld, bw_state, bw_spec)
@test traj == bw_traj

end

@testset "State-Action Recorder" begin

sim = StateActionRecorder()

actions, traj = sim(bw_plan, blocksworld, bw_state, bw_spec)
@test actions == collect(bw_plan)
@test traj == bw_traj

actions, traj = sim(bw_policy, blocksworld, bw_state, bw_spec)
@test actions == collect(bw_plan)
@test traj == bw_traj

end

@testset "Reward Accumulator" begin

sim = RewardAccumulator()

reward = sim(bw_plan, blocksworld, bw_state, bw_spec)
@test reward == -length(bw_plan)

reward = sim(bw_policy, blocksworld, bw_state, bw_spec)
@test reward == -length(bw_plan)

end

end
