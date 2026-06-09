# GPIBT

Official implementation of *"Greedy Priority Inheritance with Backtracking for Multi-agent Pathfinding Problem"*.

This repository contains Multi-Agent Path Finding (MAPF) algorithms based on Priority Inheritance with Backtracking.

## Supported Solvers

- **GPIBT-B**: Backflow-based Greedy PIBT
- **GPIBT-R**: Reduction-based Greedy PIBT

## Dependencies

- **CMake** ≥ 3.16
- **C++17** compiler (g++ or clang++)
- **OpenMP** (typically `libgomp1` or LLVM OpenMP)
- **OR-Tools** (Google OR-Tools CP-SAT solver, **v9.11** recommended) - [Installation guide](https://developers.google.com/optimization/install/cpp/source_linux)

## Clone

```bash
git clone https://github.com/tangmingkai/gpibt.git
cd gpibt

# Initialize submodules
git submodule update --init --recursive
```

## Usage

```bash
python run_one_instance.py -m <map> -s <solver> -n <num-agents> --task <task-file>
```

**Options:**

- `-m, --map`: Map file path (required)
- `-s, --solver`: Solver name (`GPIBT-B` or `GPIBT-R`, required)
- `-n, --num-agents`: Number of agents (default: 10)
- `-t, --time-limit`: Time limit in milliseconds (default: 60000)
- `-o, --output`: Output file path (default: `result.txt`)
- `--task`: Task file path (required)

**Examples:**

```bash
# Run GPIBT-B on an empty 8x8 map
python run_one_instance.py -m dataset/map/empty-8-8.map -s GPIBT-B -n 10 --task dataset/task/empty-8-8-random-1.task -o result.txt

# Run GPIBT-R on the same map
python run_one_instance.py -m dataset/map/empty-8-8.map -s GPIBT-R -n 10 --task dataset/task/empty-8-8-random-1.task -o result.txt

# Run on a 32x32 random map
python run_one_instance.py -m dataset/map/random-32-32-10.map -s GPIBT-R -n 20 --task dataset/task/random-32-32-10-random-1.task -o result.txt
```

## License

MIT License. See [LICENSE](LICENSE) for details.

## Acknowledgments

This project builds upon the following excellent open-source projects:

- [PIBT2](https://github.com/Kei18/pibt2) - Base PIBT implementation by Kei18
- [grid-pathfinding](https://github.com/Kei18/grid-pathfinding) - Graph library for pathfinding
- [OR-Tools](https://developers.google.com/optimization) - Google's optimization toolkit for CP-SAT solver
