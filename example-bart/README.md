## Example run

Ensure that all dependencies are installed according to the `Project.toml` in this directory. Assuming your current directory is the top-level Git repository, this can be done using the following commands in the Pkg REPL:

```julia
pkg> activate example-bart
pkg> dev ./
pkg> instantiate -v
```

Then the single test instance can be ran using.

```term
julia ./example-bart/landmark-test.jl
```

In this single test instance you can change which planner/heuristic combo is used to solve the problem by uncommenting/commenting on line 22-24.
To change what problem/domain is being solved change the lines 7-8

## LM Count
The Landmark Count heuristic can be found in the `landmarks` folder in `lm_count.jl`. The status manager in `landmark_status_manager.jl`

## LM Local & LM Local Smart
Both of the planners that use landmarks as intermediary goals can be found in the `planner` folder. They are called `lm_local.jl` and `lm_local_smart.jl` respectivly.
Both of these planners use some generic utilitiy functions taht can be found in  `landmark_planner_util.jl`




