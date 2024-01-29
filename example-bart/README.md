## Example run

Ensure that all dependencies are installed according to the `Project.toml` in this directory. Assuming your current directory is the top-level Git repository, this can be done using the following commands in the Pkg REPL:

```julia
pkg> activate experiments
pkg> dev ./
pkg> instantiate -v
```

Then the single test instance can be ran using.

```term
julia ./experiments/landmark-test.jl
```

In this single test instance you can change which planner/heuristic combo is used to solve the problem by uncommenting/commenting on line 22-24.
To change what problem/domain is being solved change the lines 7-8


