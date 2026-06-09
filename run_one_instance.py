#!/usr/bin/env python3
"""
Command-line MAPF testing script

Usage:
    python run_one_instance.py -m dataset/map/empty-8-8.map -s GPIBT-B --task dataset/task/empty-8-8-random-1.task
    python run_one_instance.py -m dataset/map/empty-8-8.map -s GPIBT-R -n 10 -v --task dataset/task/empty-8-8-random-1.task
    python run_one_instance.py --help
"""
import argparse
import os
import subprocess
import tempfile

def main():
    parser = argparse.ArgumentParser(description='Run a single MAPF instance')
    parser.add_argument('-m', '--map', required=True, help='Map filename')
    parser.add_argument('-s', '--solver', required=True,
                        choices=['GPIBT-B', 'GPIBT-R'],
                        help='Solver name')
    parser.add_argument('-n', '--num-agents', type=int, default=10, help='Number of agents')
    parser.add_argument('-t', '--time-limit', type=int, default=60000, help='Time limit (ms)')
    parser.add_argument('-o', '--output', default='result.txt', help='Output file path')
    parser.add_argument('--task', required=True, help='Task file (.task)')
    args = parser.parse_args()

    # Setup paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = script_dir  # Script is now at project root
    build_dir = os.path.join(project_root, "build")
    exe_path = os.path.join(build_dir, "mapf-svg-main")

    # Check executable
    if not os.path.exists(exe_path):
        print("Compiling...")
        os.makedirs(build_dir, exist_ok=True)
        subprocess.run(["cmake", ".."], cwd=build_dir, check=True)
        subprocess.run(["make", "-j4"], cwd=build_dir, check=True)

    # Task file
    task_file = args.task
    map_file = args.map

    if not os.path.exists(task_file):
        print(f"Error: Task file not found: {task_file}")
        exit(1)
    if not os.path.exists(map_file):
        print(f"Error: Map file not found: {map_file}")
        exit(1)

    # Read tasks and extract agents
    with open(task_file, 'r') as f:
        tasks = f.readlines()

    if len(tasks) < args.num_agents:
        print(f"Warning: Task count ({len(tasks)}) less than requested agents ({args.num_agents})")
        args.num_agents = len(tasks)

    agents_str = []
    for i in range(args.num_agents):
        task = tasks[i].split()
        y_s, x_s = int(task[0]), int(task[1])
        y_g, x_g = int(task[2]), int(task[3])
        agents_str.append(f"{y_s},{x_s},{y_g},{x_g}")

    # Create temporary instance file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False) as f:
        ins_file = f.name
        f.write(f"map_file={map_file}\n")
        f.write(f"task_file={task_file}\n")
        f.write(f"agents={args.num_agents}\n")
        f.write(f"seed=123\n")
        f.write(f"random_problem=0\n")
        f.write(f"max_timestep=10000000\n")
        f.write(f"max_comp_time={args.time_limit}\n")
        f.write(f"{chr(10).join(agents_str)}\n")

    # Run solver
    cmd = [exe_path, "-i", ins_file, "-s", args.solver, "-o", args.output]

    print(f"\nRunning: {' '.join(cmd)}\n")
    result = subprocess.run(cmd)

    # Cleanup
    os.unlink(ins_file)

    if result.returncode != 0:
        print(f"Solver failed with return code: {result.returncode}")
        exit(1)

if __name__ == "__main__":
    main()
