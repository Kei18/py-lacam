import argparse
from pathlib import Path

from pycam import (
    LaCAM,
    get_grid,
    get_scenario,
    save_configs_for_visualizer,
    validate_mapf_solution,
)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-m",
        "--map-file",
        type=Path,
        default=Path(__file__).parent / "assets" / "tunnel.map",
    )
    parser.add_argument(
        "-i",
        "--scen-file",
        type=Path,
        default=Path(__file__).parent / "assets" / "tunnel.scen",
    )
    parser.add_argument(
        "-N",
        "--num-agents",
        type=int,
        default=2,
    )
    parser.add_argument(
        "-o",
        "--output-file",
        type=str,
        default="output.txt",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        type=int,
        default=1,
    )
    parser.add_argument("-s", "--seed", type=int, default=0)
    parser.add_argument("-t", "--time_limit_ms", type=int, default=1000)
    parser.add_argument(
        "--flg_star",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="choose LaCAM* (default) or vanilla LaCAM",
    )

    args = parser.parse_args()

    # define problem instance
    grid = get_grid(args.map_file)
    starts, goals = get_scenario(args.scen_file, args.num_agents)

    # solve MAPF
    planner = LaCAM()
    solution = planner.solve(
        grid=grid,
        starts=starts,
        goals=goals,
        seed=args.seed,
        time_limit_ms=args.time_limit_ms,
        flg_star=args.flg_star,
        verbose=args.verbose,
    )
    validate_mapf_solution(grid, starts, goals, solution)

    # save result
    save_configs_for_visualizer(solution, args.output_file)
