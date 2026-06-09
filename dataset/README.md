# Dataset

This dataset provides grid-based maps and randomly generated task instances for benchmarking Multi-Agent Pathfinding (MAPF) algorithms. It is used in the paper *"Greedy Priority Inheritance with Backtracking for Multi-agent Pathfinding Problem"* to evaluate the PIBT, GPIBT-B, and GPIBT-R algorithms under the Single-Visit Goal (SVG) setting on biconnected graphs.

The maps are derived from the [MAPF Benchmark Set](https://movingai.com/benchmarks/mapf/index.html), where each original map has been converted to its **largest biconnected component (LBC)**. This conversion ensures that PIBT-based algorithms provide completeness guarantees.

## Directory Structure

```
dataset/
├── map/
│   ├── empty-8-8.map
│   ├── random-32-32-10.map
│   └── warehouse-20-40-10-2-2.map
└── task/
    ├── empty-8-8-random-{1..25}.task
    ├── random-32-32-10-random-{1..25}.task
    └── warehouse-20-40-10-2-2-random-{1..25}.task
```

## Maps

| Map | Dimensions | Accessible Cells (LBC) | Description |
|-----|-----------|----------------------|-------------|
| `empty-8-8` | 8 x 8 | 64 | Empty grid (fully biconnected) |
| `random-32-32-10` | 32 x 32 | 849 | Random obstacles, converted to LBC |
| `warehouse-20-40-10-2-2` | 340 x 164 | 55,391 | Structured warehouse (fully biconnected) |

The `empty-8-8` and `warehouse-20-40-10-2-2` maps are inherently biconnected, so their LBCs are identical to the original graphs. The `random-32-32-10` map has been pruned to its largest biconnected component, removing vertices outside the LBC.

### Map File Format

Maps use the standard MovingAI grid format:

```
type octile
height <H>
width <W>
map
<row 0>
<row 1>
...
<row H-1>
```

- `.` — traversable cell
- `@` — obstacle (blocked cell)

## Tasks

Each map has **25 randomly generated scenarios**. Each task file specifies the start and goal positions for all agents.

| Map | Total Agents (Lines per File) |
|-----|-----------------------------|
| `empty-8-8` | 63 |
| `random-32-32-10` | 914 |
| `warehouse-20-40-10-2-2` | 38,755 |

### Task File Format

Each `.task` file contains one line per agent (no header line). Each line has four integers:

```
<x_s> <y_s> <x_g> <y_g>
```

| Field | Description |
|-------|-------------|
| `x_s` | Column of the start position |
| `y_s` | Row of the start position |
| `x_g` | Column of the goal position |
| `y_g` | Row of the goal position |

The line index (0-indexed) serves as the implicit agent ID. For example, the first line corresponds to agent 0, the second line to agent 1, and so on.

**Example** (from `empty-8-8-random-1.task`):
```
0 1 4 6
1 3 3 5
4 0 0 1
```
- Agent 0: start at (x=0, y=1), goal at (x=4, y=6)
- Agent 1: start at (x=1, y=3), goal at (x=3, y=5)
- Agent 2: start at (x=4, y=0), goal at (x=0, y=1)

### Varying the Number of Agents

To run experiments with a specific number of agents *k*, simply use the first *k* lines of a task file. The agents are ordered such that taking the first *k* lines yields a valid sub-problem with *k* agents.

### Coordinate System

- Origin (0, 0) is at the **top-left** corner.
- **x** (column) increases to the right.
- **y** (row) increases downward.



The benchmark maps and scenarios were originally introduced in:

```bibtex
@inproceedings{stern2019mapf,
  title={Multi-Agent Pathfinding: Definitions, Variants, and Benchmarks},
  author={Stern, Ron and Sturtevant, Nathan R. and Felner, Ariel and Koenig, Sven and others},
  booktitle={Proceedings of the International Symposium on Combinatorial Search (SoCS)},
  year={2019}
}
```
