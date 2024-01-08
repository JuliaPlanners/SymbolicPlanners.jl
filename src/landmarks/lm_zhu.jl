export compute_propagation

mutable struct graph_node
  labels::Set{Int}
  graph_node() = new()
end

# function deepcopy(node::graph_node)
#  new_node = graph_node()
#  new_node.labels = deepcopy(node.labels)
#  return new_node
# end

# function deepcopy_vector(vec::Vector{graph_node})
#  return [deepcopy(node) for node in vec]
# end

function compute_propagation(pgraph::PlanningGraph)
  triggers = Vector{Vector{Int}}()

  n_conditions = length(pgraph.conditions)
  resize!(triggers, n_conditions)

  for (i, conditions) in enumerate(pgraph.cond_children)
    triggers[i] = Vector{Int}()

    for tuple in conditions
      idx_action, idx_precond = tuple

      push!(triggers[i], idx_action)
    end
  end

  return triggers
end

function effect_empty(layer::Vector{graph_node}, effect::Vector{Int})
  for cond in effect
    if(isempty(layer[cond].labels))
      return false
    end
  end
  return true
end

function precond_empty(layer::Vector{graph_node}, precond::Vector{Vector{Int}})
  for outer in precond
    for cond in outer
      if (isempty(layer[cond].labels))
        return false
      end
    end
  end
  return true
end


function apply_action_and_propagate(layer::Vector{graph_node}, pgraph::PlanningGraph, action::Int, next_layer::Vector{graph_node}, triggers::Vector{Vector{Int}})
  result = Set{Int}()

  precond_union = union_preconditions(layer, pgraph.act_parents[action])

  for effect in pgraph.act_children[action]
    precond_effect = copy(precond_union)
    union_eff = union(precond_effect, effect)

    union_effects = union_effect(layer, pgraph.act_children[action])
    union!(precond_effect, union_effects)

    if (labels_propagated(next_layer[effect].labels, precond_effect, effect)) 
      push!(result, effect)
      next_layer[effect].labels = union_eff
    end

  end

  return result
end

function labels_propagated(old_labels::Set{Int}, new_labels::Set{Int}, condition::Int)
  copy_old = deepcopy(old_labels)

  if(!isempty(old_labels))
    intersect!(old_labels, new_labels)
  else
    old_labels = new_labels
  end

  push!(old_labels, condition)

  return length(old_labels) != length(copy_old)
end

function union_preconditions(layer::Vector{graph_node}, precond::Vector{Vector{Int}})
  result = Set{Int}()

  for out in precond
    for cond in out
      union!(result, layer[cond].labels)
    end
  end
  return result
end

function union_effect(layer::Vector{graph_node}, effect::Vector{Int})
  result = Set{Int}()

  for cond in effect
    union!(result, layer[cond].labels)
  end

  return result
end

function graph_label(pgraph::PlanningGraph, domain::Domain, state::State)

  n_conditions = length(pgraph.conditions)
  triggers = compute_propagation(pgraph)
  
  triggered = Set{Int}()

  init_idxs = pgraph_init_idxs(pgraph, domain, state)
  # prop_layer::Vector{graph_node}()
  prop_layer =  Vector{graph_node}()

  for i in 1:n_conditions
    node = graph_node()
    node.labels = Set{Int}()
    push!(prop_layer, node)
  end

  for i in findall(init_idxs)
    push!(prop_layer[i].labels, i)
  end

  for (i, conditions) in enumerate(pgraph.conditions)
    for action_id in triggers[i]
      push!(triggered, action_id)
    end
  end

  changes = true

  while(changes)
    next_layer = deepcopy(prop_layer)
    next_trigger = Set()
    changes = false
    for i in triggered
      # precond::Vector{Vector{Int}}
      precond = pgraph.act_parents[i]

      if (precond_empty(prop_layer, precond))
         
        changed = apply_action_and_propagate(prop_layer, pgraph, i, next_layer, triggers)
        if (!isempty(changed))
          
          changes = true
          for j in changed
            for val in triggers[j]
              push!(next_trigger, val)
            end
          end
        end
      end

    end
    prop_layer = next_layer
    triggered = next_trigger
  end

  n_action = length(pgraph.actions)
 
  res = Set{Int}()

  for out in pgraph.act_parents[n_action]
    for i in out
      for j in prop_layer[i].labels
      push!(res, j)
      end
    end
  end

  return prop_layer
end