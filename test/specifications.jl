@testset "Specifications" begin

using SymbolicPlanners: get_goal_terms, set_goal_terms

@testset "MinStepsGoal" begin

# Test construction
spec = MinStepsGoal(bw_problem.goal)
@test Specification(bw_problem) == spec

# Test hashing and equality
@test hash(MinStepsGoal(bw_problem.goal)) == hash(spec)
@test MinStepsGoal(bw_problem.goal) == spec
@test MinStepsGoal(pddl"(true)") != spec

# Test goal satisfaction
@test is_goal(spec, blocksworld, bw_state) == false
goal_state = copy(bw_state)
goal_state[pddl"(clear c)"] = true
goal_state[pddl"(on c a)"] = true
goal_state[pddl"(on a b)"] = true
goal_state[pddl"(ontable b)"] = true
@test is_goal(spec, blocksworld, goal_state) == true

# Test constraint violation
@test is_violated(spec, blocksworld, bw_state) == false

# Test costs and rewards
action = pddl"(pick-up a)"
next_state = PDDL.transition(blocksworld, bw_state, action)
@test get_cost(spec, blocksworld, bw_state, action, next_state) == 1.0
@test get_reward(spec, blocksworld, bw_state, action, next_state) == -1.0
@test get_discount(spec) == 1.0 
@test has_action_cost(spec) == false

# Test goal term accessors
@test get_goal_terms(spec) == PDDL.flatten_conjs(bw_problem.goal)
new_spec = set_goal_terms(spec, Term[pddl"(true)"])
@test get_goal_terms(new_spec) == Term[pddl"(true)"]

end

@testset "MinMetricGoal" begin

# Test construction
metric = PDDL.get_metric(zt_problem).args[1]
spec = MinMetricGoal(zt_problem.goal, metric)
@test Specification(zt_problem) == spec

# Test hashing and equality
@test hash(MinMetricGoal(zt_problem.goal, metric)) == hash(spec)
@test MinMetricGoal(zt_problem.goal, metric) == spec
@test MinMetricGoal(pddl"(true)", metric) != spec

# Test goal satisfaction
@test is_goal(spec, zeno_travel, zt_state) == false
goal_state = copy(zt_state)
goal_state[pddl"(at plane1 city1)"] = true
goal_state[pddl"(at person1 city2)"] = true
goal_state[pddl"(at person2 city0)"] = true
@test is_goal(spec, zeno_travel, goal_state) == true

# Test constraint violation
@test is_violated(spec, zeno_travel, zt_state) == false

# Test costs and rewards
action = pddl"(fly plane1 city0 city2)"
next_state = PDDL.transition(zeno_travel, zt_state, action)
fuel_used = zt_state[pddl"(slow-burn plane1)"] * zt_state[pddl"(distance city0 city2)"]
cost = 4.0 + 5.0 * fuel_used
@test get_cost(spec, zeno_travel, zt_state, action, next_state) == cost
@test get_reward(spec, zeno_travel, zt_state, action, next_state) == -cost
@test get_discount(spec) == 1.0 
@test has_action_cost(spec) == false

# Test goal term accessors
@test get_goal_terms(spec) == PDDL.flatten_conjs(zt_problem.goal)
new_spec = set_goal_terms(spec, Term[pddl"(true)"])
@test get_goal_terms(new_spec) == Term[pddl"(true)"]

end

@testset "MaxMetricGoal" begin

# Test construction
metric = PDDL.get_metric(zt_problem).args[1]
spec = MaxMetricGoal(zt_problem.goal, metric)

# Test hashing and equality
@test hash(MaxMetricGoal(zt_problem.goal, metric)) == hash(spec)
@test MaxMetricGoal(zt_problem.goal, metric) == spec
@test MaxMetricGoal(pddl"(true)", metric) != spec

# Test goal satisfaction
@test is_goal(spec, zeno_travel, zt_state) == false
goal_state = copy(zt_state)
goal_state[pddl"(at plane1 city1)"] = true
goal_state[pddl"(at person1 city2)"] = true
goal_state[pddl"(at person2 city0)"] = true
@test is_goal(spec, zeno_travel, goal_state) == true

# Test constraint violation
@test is_violated(spec, zeno_travel, zt_state) == false

# Test costs and rewards
action = pddl"(fly plane1 city0 city2)"
next_state = PDDL.transition(zeno_travel, zt_state, action)
fuel_used = zt_state[pddl"(slow-burn plane1)"] * zt_state[pddl"(distance city0 city2)"]
reward = 4.0 + 5.0 * fuel_used
@test get_cost(spec, zeno_travel, zt_state, action, next_state) == -reward
@test get_reward(spec, zeno_travel, zt_state, action, next_state) == reward

# Test goal term accessors
@test get_goal_terms(spec) == PDDL.flatten_conjs(zt_problem.goal)
new_spec = set_goal_terms(spec, Term[pddl"(true)"])
@test get_goal_terms(new_spec) == Term[pddl"(true)"]

end

@testset "StateConstrainedGoal" begin

# Test construction
goal_spec = MinStepsGoal(wgc_problem.goal)
constraints = wgc_problem.constraints
spec = StateConstrainedGoal(goal_spec, constraints)
@test StateConstrainedGoal(wgc_problem) == spec

# Test hashing and equality
@test hash(StateConstrainedGoal(goal_spec, constraints)) == hash(spec)
@test StateConstrainedGoal(goal_spec, constraints) == spec
@test StateConstrainedGoal(goal_spec, pddl"(true)") != spec

# Test goal satisfaction
@test is_goal(spec, wgc_domain, wgc_state) == false
goal_state = copy(wgc_state)
goal_state[pddl"(cabbage-at right)"] = true
goal_state[pddl"(goat-at right)"] = true
goal_state[pddl"(wolf-at right)"] = true
goal_state[pddl"(boat-at right)"] = true
@test is_goal(spec, wgc_domain, goal_state) == true

# Test constraint violation
@test is_violated(spec, wgc_domain, wgc_state) == false
next_state = PDDL.transition(wgc_domain, wgc_state, pddl"(ship-wolf left right)")
@test is_violated(spec, wgc_domain, next_state) == true
next_state = PDDL.transition(wgc_domain, wgc_state, pddl"(ship-cabbage left right)")
@test is_violated(spec, wgc_domain, next_state) == true
next_state = PDDL.transition(wgc_domain, wgc_state, pddl"(ship-goat left right)")
@test is_violated(spec, wgc_domain, next_state) == false

# Test costs and rewards
action = pddl"(ship-goat left right)"
next_state = PDDL.transition(wgc_domain, wgc_state, action)
@test get_cost(spec, wgc_domain, wgc_state, action, next_state) == 1.0
@test get_reward(spec, wgc_domain, wgc_state, action, next_state) == -1.0
@test get_discount(spec) == 1.0
@test has_action_cost(spec) == false

# Test goal term accessors
@test get_goal_terms(spec) == PDDL.flatten_conjs(wgc_problem.goal)
new_spec = set_goal_terms(spec, Term[pddl"(true)"])
@test get_goal_terms(new_spec) == Term[pddl"(true)"]

end

@testset "BackwardSearchGoal" begin

# Test construction
goal_spec = MinStepsGoal(bw_problem.goal)
spec = BackwardSearchGoal(goal_spec, bw_state)
@test BackwardSearchGoal(goal_spec, blocksworld, bw_state) == spec

# Test hashing and equality
new_goal_spec = MinStepsGoal(bw_problem.goal)
@test hash(BackwardSearchGoal(new_goal_spec, bw_state)) == hash(spec)
@test BackwardSearchGoal(new_goal_spec, bw_state) == spec
@test BackwardSearchGoal(MinStepsGoal([]), bw_state) != spec

# Test goal satisfaction
@test is_goal(spec, blocksworld, bw_state) == true
goal_state = goalstate(blocksworld, PDDL.get_objtypes(bw_state), get_goal_terms(spec))
@test is_goal(spec, blocksworld, goal_state) == false

# Test constraint violation
@test is_violated(spec, blocksworld, bw_state) == false

# Test costs and rewards
action = pddl"(pick-up a)"
next_state = PDDL.transition(blocksworld, bw_state, action)
@test get_cost(spec, blocksworld, bw_state, action, next_state) == 1.0
@test get_reward(spec, blocksworld, bw_state, action, next_state) == -1.0
@test get_discount(spec) == 1.0
@test has_action_cost(spec) == false

# Test goal term accessors
@test get_goal_terms(spec) == PDDL.flatten_conjs(bw_problem.goal)
new_spec = set_goal_terms(spec, Term[pddl"(true)"])
@test get_goal_terms(new_spec) == Term[pddl"(true)"]

end

@testset "ActionGoal" begin

# Test construction
goal_action = pddl"(stack ?x b)"
constraints = @pddl("(ontable b)", "(clear ?x)")
spec = ActionGoal(goal_action, constraints)
zero_cost_spec = ActionGoal(goal_action, constraints, 0.0)
@test ActionGoal(goal_action, constraints, 1) == spec

# Test hashing and equality
@test hash(ActionGoal(goal_action, constraints)) == hash(spec)
@test ActionGoal(goal_action, constraints) == spec
@test ActionGoal(goal_action, pddl"(true)") != spec
@test ActionGoal(goal_action, constraints, 0.0) != spec

# Test goal satisfaction
@test_throws Exception is_goal(spec, blocksworld, bw_state)
@test is_goal(spec, blocksworld, bw_state, pddl"(stack a b)") == true
@test is_goal(spec, blocksworld, bw_state, pddl"(stack c b)") == true
@test is_goal(spec, blocksworld, bw_state, pddl"(pick-up a)") == false
@test is_goal(spec, blocksworld, bw_state, pddl"(stack c a)") == false
non_goal_state = copy(bw_state)
non_goal_state[pddl"(clear a)"] = false
@test is_goal(spec, blocksworld, non_goal_state, pddl"(stack a b)") == false
non_goal_state[pddl"(clear a)"] = true
non_goal_state[pddl"(ontable b)"] = false
@test is_goal(spec, blocksworld, non_goal_state, pddl"(stack a b)") == false

# Test constraint violation
@test is_violated(spec, blocksworld, bw_state) == false

# Test action costs
@test has_action_cost(spec) == true
@test get_action_cost(spec, pddl"(pick-up b)") == 1.0
@test get_action_cost(spec, pddl"(put-down b)") == 1.0
@test get_action_cost(spec, pddl"(stack a b)") == 1.0
@test get_action_cost(spec, pddl"(unstack a b)") == 1.0

# Test costs and rewards
action = pddl"(pick-up a)"
next_state = PDDL.transition(blocksworld, bw_state, action)
@test get_cost(spec, blocksworld, bw_state, action, next_state) == 1.0
@test get_reward(spec, blocksworld, bw_state, action, next_state) == -1.0
@test get_cost(zero_cost_spec, blocksworld, bw_state, action, next_state) == 0.0
@test get_reward(zero_cost_spec, blocksworld, bw_state, action, next_state) == 0.0
@test get_discount(spec) == 1.0

# Test goal term accessors
@test get_goal_terms(spec) == Term[pddl"(do-action (stack ?x b) (and (ontable b) (clear ?x)))"]
new_spec = set_goal_terms(spec, Term[pddl"(do-action (stack a b))"])
@test get_goal_terms(new_spec) == Term[pddl"(do-action (stack a b))"]
@test new_spec.action == pddl"(stack a b)"
@test isempty(new_spec.constraints)

end

@testset "DiscountedReward" begin

# Test construction
goal_spec = MinStepsGoal(pddl"(on a b)")
spec = DiscountedReward(goal_spec, 0.9)
@test discounted(goal_spec, 0.9) == spec
new_spec = DiscountedReward(goal_spec)
@test discounted(goal_spec, 1.0) == new_spec

# Test hashing and equality
new_goal_spec = MinStepsGoal(pddl"(on a b)")
@test hash(DiscountedReward(new_goal_spec, 0.9)) == hash(spec)
@test DiscountedReward(new_goal_spec, 0.9) == spec
@test DiscountedReward(goal_spec, 0.8) != spec

# Test goal satisfaction
@test is_goal(spec, blocksworld, bw_state) == false
goal_state = copy(bw_state)
goal_state[pddl"(on a b)"] = true
@test is_goal(spec, blocksworld, goal_state) == true

# Test constraint violation
@test is_violated(spec, blocksworld, bw_state) == false

# Test costs and rewards
action = pddl"(pick-up a)"
next_state = PDDL.transition(blocksworld, bw_state, action)
@test get_cost(spec, blocksworld, bw_state, action, next_state) == 1.0
@test get_reward(spec, blocksworld, bw_state, action, next_state) == -1.0
@test has_action_cost(spec) == false

# Test discounting
@test get_discount(spec) == 0.9
new_spec = discounted(spec, 0.9)
@test get_discount(new_spec) == 0.9 * 0.9

# Test goal term accessors
@test get_goal_terms(spec) == PDDL.flatten_conjs(pddl"(on a b)")
new_spec = set_goal_terms(spec, Term[pddl"(true)"])
@test get_goal_terms(new_spec) == Term[pddl"(true)"]

end

@testset "GoalReward" begin

# Test construction
spec = GoalReward(pddl"(on a b)", 1.0, 0.9)

# Test hashing and equality
@test hash(GoalReward(pddl"(on a b)", 1.0, 0.9)) == hash(spec)
@test GoalReward(pddl"(on a b)", 1.0, 0.9) == spec
@test GoalReward(pddl"(on a b)", 1.0, 0.8) != spec

# Test goal satisfaction
@test is_goal(spec, blocksworld, bw_state) == false
goal_state = copy(bw_state)
goal_state[pddl"(on a b)"] = true
@test is_goal(spec, blocksworld, goal_state) == true

# Test constraint violation
@test is_violated(spec, blocksworld, bw_state) == false

# Test costs and rewards
actions = @pddl("(pick-up a)", "(stack a b)")
next_state = PDDL.transition(blocksworld, bw_state, actions[1])
goal_state = PDDL.transition(blocksworld, next_state, actions[2])
@test get_cost(spec, blocksworld, bw_state, actions[1], next_state) == 0.0
@test get_reward(spec, blocksworld, bw_state, actions[1], next_state) == 0.0
@test get_cost(spec, blocksworld, next_state, actions[2], goal_state) == -1.0
@test get_reward(spec, blocksworld, next_state, actions[2], goal_state) == 1.0
@test has_action_cost(spec) == false

# Test discounting
@test get_discount(spec) == 0.9
new_spec = discounted(spec, 0.9)
@test get_discount(new_spec) == 0.9 * 0.9

# Test goal term accessors
@test get_goal_terms(spec) == [pddl"(on a b)"]
new_spec = set_goal_terms(spec, Term[pddl"(true)"])
@test get_goal_terms(new_spec) == Term[pddl"(true)"]

end

@testset "BonusGoalReward" begin

# Test construction
goal_spec = MinStepsGoal(pddl"(on a b)")
spec = BonusGoalReward(goal_spec, 10.0, 0.9)
@test BonusGoalReward(goal_spec, 10.0, 0.9) == spec

# Test hashing and equality
new_goal_spec = MinStepsGoal(pddl"(on a b)")
@test hash(BonusGoalReward(new_goal_spec, 10.0, 0.9)) == hash(spec)
@test BonusGoalReward(new_goal_spec, 10.0, 0.9) == spec
@test BonusGoalReward(new_goal_spec, 10.0, 0.8) != spec

# Test goal satisfaction
@test is_goal(spec, blocksworld, bw_state) == false
goal_state = copy(bw_state)
goal_state[pddl"(on a b)"] = true
@test is_goal(spec, blocksworld, goal_state) == true

# Test constraint violation
@test is_violated(spec, blocksworld, bw_state) == false

# Test costs and rewards
actions = @pddl("(pick-up a)", "(stack a b)")
next_state = PDDL.transition(blocksworld, bw_state, actions[1])
goal_state = PDDL.transition(blocksworld, next_state, actions[2])
@test get_cost(spec, blocksworld, bw_state, actions[1], next_state) == 1.0
@test get_reward(spec, blocksworld, bw_state, actions[1], next_state) == -1.0
@test get_cost(spec, blocksworld, next_state, actions[2], goal_state) == -9.0
@test get_reward(spec, blocksworld, next_state, actions[2], goal_state) == 9.0
@test has_action_cost(spec) == false

# Test discounting
@test get_discount(spec) == 0.9
new_spec = discounted(spec, 0.9)
@test get_discount(new_spec) == 0.9 * 0.9

# Test goal term accessors
@test get_goal_terms(spec) == [pddl"(on a b)"]
new_spec = set_goal_terms(spec, Term[pddl"(true)"])
@test get_goal_terms(new_spec) == Term[pddl"(true)"]

end

@testset "MultiGoalReward" begin

# Test construction
goals = [pddl"(on a b)", pddl"(on b c)"]
rewards = [1.0, 2.0]
spec = MultiGoalReward(goals, rewards, 0.9)

# Test hashing and equality
@test hash(MultiGoalReward(copy(goals), copy(rewards), 0.9)) == hash(spec)
@test MultiGoalReward(copy(goals), copy(rewards), 0.9) == spec
@test MultiGoalReward(goals, rewards, 0.8) != spec

# Test goal satisfaction
@test is_goal(spec, blocksworld, bw_state) == false
goal_state_1 = copy(bw_state)
goal_state_1[pddl"(on a b)"] = true
@test is_goal(spec, blocksworld, goal_state_1) == true
goal_state_2 = copy(bw_state)
goal_state_2[pddl"(on b c)"] = true
@test is_goal(spec, blocksworld, goal_state_2) == true

# Test constraint violation
@test is_violated(spec, blocksworld, bw_state) == false

# Test costs and rewards
actions_1 = @pddl("(pick-up a)", "(stack a b)")
next_state_1 = PDDL.transition(blocksworld, bw_state, actions_1[1])
goal_state_1 = PDDL.transition(blocksworld, next_state_1, actions_1[2])
@test get_cost(spec, blocksworld, bw_state, actions_1[1], next_state_1) == 0.0
@test get_reward(spec, blocksworld, bw_state, actions_1[1], next_state_1) == 0.0
@test get_cost(spec, blocksworld, next_state_1, actions_1[2], goal_state_1) == -1.0
@test get_reward(spec, blocksworld, next_state_1, actions_1[2], goal_state_1) == 1.0

actions_2 = @pddl("(pick-up b)", "(stack b c)")
next_state_2 = PDDL.transition(blocksworld, bw_state, actions_2[1])
goal_state_2 = PDDL.transition(blocksworld, next_state_2, actions_2[2])
@test get_cost(spec, blocksworld, bw_state, actions_2[1], next_state_2) == 0.0
@test get_reward(spec, blocksworld, bw_state, actions_2[1], next_state_2) == 0.0
@test get_cost(spec, blocksworld, next_state_2, actions_2[2], goal_state_2) == -2.0
@test get_reward(spec, blocksworld, next_state_2, actions_2[2], goal_state_2) == 2.0

@test has_action_cost(spec) == false

# Test discounting
@test get_discount(spec) == 0.9
new_spec = discounted(spec, 0.9)
@test get_discount(new_spec) == 0.9 * 0.9

# Test goal term accessors
@test get_goal_terms(spec) == [Compound(:or, goals)]
new_spec = set_goal_terms(spec, @pddl("(true)", "(true)"))
@test get_goal_terms(new_spec) == [Compound(:or, @pddl("(true)", "(true)"))]

end

@testset "MinActionCosts" begin

# Test construction
costs = (var"pick-up" = 2.0, var"put-down" = 1.0, stack = 1.5, unstack = 2.0)
spec_1 = MinActionCosts(bw_problem.goal, costs)
spec_2 = MinActionCosts(bw_problem.goal; costs...)
spec_3 = MinActionCosts(bw_problem.goal, keys(costs), values(costs))
spec_4 = MinActionCosts(bw_problem.goal, collect(keys(costs)), collect(costs))
@test spec_1 == spec_2 == spec_3 == spec_4

# Test action cost inference
zt_infer_spec = MinActionCosts(zeno_travel, zt_problem)
@test zt_infer_spec isa MinActionCosts{Dict{Term, Float64}}

# Test hashing and equality
spec = spec_1
@test hash(MinActionCosts(bw_problem.goal, costs)) == hash(spec)
@test MinActionCosts(bw_problem.goal, costs) == spec
@test MinActionCosts(pddl"(true)", costs) != spec

# Test goal satisfaction
@test is_goal(spec, blocksworld, bw_state) == false
goal_state = copy(bw_state)
goal_state[pddl"(clear c)"] = true
goal_state[pddl"(on c a)"] = true
goal_state[pddl"(on a b)"] = true
goal_state[pddl"(ontable b)"] = true
@test is_goal(spec, blocksworld, goal_state) == true

# Test constraint violation
@test is_violated(spec, blocksworld, bw_state) == false

# Test action costs
@test has_action_cost(spec) == true
@test get_action_cost(spec, pddl"(pick-up b)") == 2.0
@test get_action_cost(spec, pddl"(put-down b)") == 1.0
@test get_action_cost(spec, pddl"(stack a b)") == 1.5
@test get_action_cost(spec, pddl"(unstack a b)") == 2.0

@test has_action_cost(zt_infer_spec) == true
fuel_used = zt_state[pddl"(slow-burn plane1)"] * zt_state[pddl"(distance city0 city2)"]
@test get_action_cost(zt_infer_spec, pddl"(fly plane1 city0 city2)") == 4.0 + 5.0 * fuel_used

# Test costs and rewards
action = pddl"(pick-up b)"
next_state = PDDL.transition(blocksworld, bw_state, action)
@test get_cost(spec, blocksworld, bw_state, action, next_state) == 2.0
@test get_reward(spec, blocksworld,bw_state, action, next_state) == -2.0
@test get_discount(spec) == 1.0 

# Test goal term accessors
@test get_goal_terms(spec) == PDDL.flatten_conjs(bw_problem.goal)
new_spec = set_goal_terms(spec, Term[pddl"(true)"])
@test get_goal_terms(new_spec) == Term[pddl"(true)"]

end

@testset "ExtraActionCosts" begin

# Test construction
goal_spec = GoalReward(pddl"(on a b)", 10.0, 0.9)
costs = (var"pick-up" = 2.0, var"put-down" = 1.0, stack = 1.5, unstack = 2.0)
spec_1 = ExtraActionCosts(goal_spec, costs)
spec_2 = ExtraActionCosts(goal_spec; costs...)
spec_3 = ExtraActionCosts(goal_spec, keys(costs), values(costs))
spec_4 = ExtraActionCosts(goal_spec, collect(keys(costs)), collect(costs))
@test spec_1 == spec_2 == spec_3 == spec_4

# Test hashing and equality
spec = spec_1
new_goal_spec = GoalReward(pddl"(on a b)", 10.0, 0.9)
@test hash(ExtraActionCosts(new_goal_spec, costs)) == hash(spec)
@test ExtraActionCosts(new_goal_spec, costs) == spec
@test ExtraActionCosts(new_goal_spec, Dict{Term, Float64}()) != spec

# Test goal satisfaction
@test is_goal(spec, blocksworld, bw_state) == false
goal_state = copy(bw_state)
goal_state[pddl"(on a b)"] = true
@test is_goal(spec, blocksworld, goal_state) == true

# Test constraint violation
@test is_violated(spec, blocksworld, bw_state) == false

# Test action costs
@test has_action_cost(spec) == true
@test get_action_cost(spec, pddl"(pick-up b)") == 2.0
@test get_action_cost(spec, pddl"(put-down b)") == 1.0
@test get_action_cost(spec, pddl"(stack a b)") == 1.5
@test get_action_cost(spec, pddl"(unstack a b)") == 2.0

# Test costs and rewards
actions = @pddl("(pick-up a)", "(stack a b)")
next_state = PDDL.transition(blocksworld, bw_state, actions[1])
goal_state = PDDL.transition(blocksworld, next_state, actions[2])
@test get_cost(spec, blocksworld, bw_state, actions[1], next_state) == 2.0
@test get_reward(spec, blocksworld, bw_state, actions[1], next_state) == -2.0
@test get_cost(spec, blocksworld, next_state, actions[2], goal_state) == -8.5
@test get_reward(spec, blocksworld, next_state, actions[2], goal_state) == 8.5

# Test discounting
@test get_discount(spec) == 0.9
new_spec = discounted(spec, 0.9)
@test get_discount(new_spec) == 0.9 * 0.9

# Test goal term accessors
@test get_goal_terms(spec) == [pddl"(on a b)"]
new_spec = set_goal_terms(spec, Term[pddl"(true)"])
@test get_goal_terms(new_spec) == Term[pddl"(true)"]

end

@testset "MinPerAgentActionCosts" begin

# Test construction
costs_plane = (board = 0., debark = 0., refuel = 0.5, fly = 1., zoom = 2.)
costs_person1 = (board = 1., debark = 1., refuel = 0., fly = 0., zoom = 0.)
costs_person2 = (board = 2., debark = 2., refuel = 0., fly = 0., zoom = 0.)
costs = (plane1 = costs_plane, person1 = costs_person1, person2 = costs_person2)
spec = MinPerAgentActionCosts(zt_problem.goal, costs)
@test MinPerAgentActionCosts(zt_problem.goal, costs, 1) == spec

# Test hashing and equality
@test hash(MinPerAgentActionCosts(zt_problem.goal, costs)) == hash(spec)
@test MinPerAgentActionCosts(zt_problem.goal, costs) == spec
@test MinPerAgentActionCosts(zt_problem.goal, costs, 2) != spec

# Test goal satisfaction
@test is_goal(spec, zeno_travel, zt_state) == false
goal_state = copy(zt_state)
goal_state[pddl"(at plane1 city1)"] = true
goal_state[pddl"(at person1 city2)"] = true
goal_state[pddl"(at person2 city0)"] = true
@test is_goal(spec, zeno_travel, goal_state) == true

# Test constraint violation
@test is_violated(spec, zeno_travel, zt_state) == false

# Test action costs
@test has_action_cost(spec) == true
@test get_action_cost(spec, pddl"(board plane1 city0)") == 0.0
@test get_action_cost(spec, pddl"(debark plane1 city0)") == 0.0
@test get_action_cost(spec, pddl"(refuel plane1 city 0)") == 0.5
@test get_action_cost(spec, pddl"(fly plane1 city0 city2)") == 1.0
@test get_action_cost(spec, pddl"(zoom plane1 city0 city2)") == 2.0
@test get_action_cost(spec, pddl"(board person1 city0)") == 1.0
@test get_action_cost(spec, pddl"(debark person1 city0)") == 1.0
@test get_action_cost(spec, pddl"(board person2 city0)") == 2.0
@test get_action_cost(spec, pddl"(debark person2 city0)") == 2.0

# Test costs and rewards
action = pddl"(fly plane1 city0 city2)"
next_state = PDDL.transition(zeno_travel, zt_state, action)
@test get_cost(spec, zeno_travel, zt_state, action, next_state) == 1.0
@test get_reward(spec, zeno_travel, zt_state, action, next_state) == -1.0
action = pddl"(refuel plane1 city0)"
next_state = PDDL.transition(zeno_travel, zt_state, action)
@test get_cost(spec, zeno_travel, zt_state, action, next_state) == 0.5
@test get_reward(spec, zeno_travel, zt_state, action, next_state) == -0.5
@test get_discount(spec) == 1.0

# Test goal term accessors
@test get_goal_terms(spec) == PDDL.flatten_conjs(zt_problem.goal)
new_spec = set_goal_terms(spec, Term[pddl"(true)"])
@test get_goal_terms(new_spec) == Term[pddl"(true)"]

end

@testset "ExtraPerAgentActionCosts" begin

# Test construction
goal_spec = GoalReward(pddl"(at plane1 city2)", 10.0, 0.9)
costs_plane = (board = 0., debark = 0., refuel = 0.5, fly = 1., zoom = 2.)
costs_person1 = (board = 1., debark = 1., refuel = 0., fly = 0., zoom = 0.)
costs_person2 = (board = 2., debark = 2., refuel = 0., fly = 0., zoom = 0.)
costs = (plane1 = costs_plane, person1 = costs_person1, person2 = costs_person2)
spec = ExtraPerAgentActionCosts(goal_spec, costs)
@test ExtraPerAgentActionCosts(goal_spec, costs, 1) == spec

# Test hashing and equality
new_goal_spec = GoalReward(pddl"(at plane1 city2)", 10.0, 0.9)
@test hash(ExtraPerAgentActionCosts(new_goal_spec, costs)) == hash(spec)
@test ExtraPerAgentActionCosts(new_goal_spec, costs) == spec
@test ExtraPerAgentActionCosts(new_goal_spec, costs, 2) != spec

# Test goal satisfaction
@test is_goal(spec, zeno_travel, zt_state) == false
goal_state = copy(zt_state)
goal_state[pddl"(at plane1 city2)"] = true
@test is_goal(spec, zeno_travel, goal_state) == true

# Test constraint violation
@test is_violated(spec, zeno_travel, zt_state) == false

# Test action costs
@test has_action_cost(spec) == true
@test get_action_cost(spec, pddl"(board plane1 city0)") == 0.0
@test get_action_cost(spec, pddl"(debark plane1 city0)") == 0.0
@test get_action_cost(spec, pddl"(refuel plane1 city 0)") == 0.5
@test get_action_cost(spec, pddl"(fly plane1 city0 city2)") == 1.0
@test get_action_cost(spec, pddl"(zoom plane1 city0 city2)") == 2.0
@test get_action_cost(spec, pddl"(board person1 city0)") == 1.0
@test get_action_cost(spec, pddl"(debark person1 city0)") == 1.0
@test get_action_cost(spec, pddl"(board person2 city0)") == 2.0
@test get_action_cost(spec, pddl"(debark person2 city0)") == 2.0

# Test costs and rewards
action = pddl"(fly plane1 city0 city2)"
next_state = PDDL.transition(zeno_travel, zt_state, action)
@test get_cost(spec, zeno_travel, zt_state, action, next_state) == -9.0
@test get_reward(spec, zeno_travel, zt_state, action, next_state) == 9.0
action = pddl"(refuel plane1 city0)"
next_state = PDDL.transition(zeno_travel, zt_state, action)
@test get_cost(spec, zeno_travel, zt_state, action, next_state) == 0.5
@test get_reward(spec, zeno_travel, zt_state, action, next_state) == -0.5

# Test discounting
@test get_discount(spec) == 0.9
new_spec = discounted(spec, 0.9)
@test get_discount(new_spec) == 0.9 * 0.9

# Test goal term accessors
@test get_goal_terms(spec) == [pddl"(at plane1 city2)"]
new_spec = set_goal_terms(spec, Term[pddl"(true)"])
@test get_goal_terms(new_spec) == Term[pddl"(true)"]

end

end
