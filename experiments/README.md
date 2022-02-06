## Performance Experiments

Ensure that all dependencies are installed according to the `Project.toml` in this directory. Assuming your current directory is the top-level Git repository, this can be done using the following commands in the Pkg REPL:

```julia
pkg> activate experiments
pkg> dev ./
pkg> instantiate -v
```

For logical / propositional domains, run [`logical-experiments.jl`](logical-experiments.jl). To run experiments for the external planners, install [Pyperplan](https://github.com/aibasel/pyperplan) and [FastDownward](https://www.fast-downward.org/). Then, set `ENV["PYTHON"]` to the Python executable path for the Python environment where Pyperplan is installed, and `ENV["FD_PATH"]` to the path for `fast_downward.py`.

For numeric domains, run [`numeric-experiments.jl`](numeric-experiments.jl). To run experiments for the external planners, install [ENHSP](https://sites.google.com/view/enhsp/). Then, set `ENV["JAVA"]` to the path to the Java executable, and `ENV["ENHSP_PATH"]` for the path to `enhsp.jar`.

Domain and problem files are from [IPC 2000](https://github.com/potassco/pddl-instances/tree/master/ipc-2000) and [IPC 2002](https://github.com/potassco/pddl-instances/tree/master/ipc-2002), as archived by [`pddl-instances`](https://github.com/potassco/pddl-instances).
