from pathlib import Path

import numpy as np

from pycam.lacam import LaCAM
from pycam.mapf_utils import (
    Config,
    get_sum_of_loss,
    is_valid_mapf_solution,
    save_configs_for_visualizer,
)

DIR = Path(__file__).parent


def test_LaCAM():
    grid = np.full((2, 3), True)
    starts = Config([(0, 0), (0, 2)])
    goals = Config([(0, 2), (0, 0)])
    planner = LaCAM()
    solution = planner.solve(grid=grid, starts=starts, goals=goals, flg_star=True)
    assert is_valid_mapf_solution(grid, starts, goals, solution)
    assert get_sum_of_loss(solution) == 6
    save_configs_for_visualizer(solution, DIR / "local" / "lacam-example.txt")
