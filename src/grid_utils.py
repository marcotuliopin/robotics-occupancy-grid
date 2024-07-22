import numpy as np
from .interfaces import *

def get_cell_idx(
    p: Point, grid_size: int, cell_size: float, center: bool = True
) -> Point:
    """
    Calculate the cell index in a grid for a given point.

    Args:
        p (Point): The point to calculate the cell index for.
        grid_size (int): The size of the grid.
        cell_size (float): The size of each cell in the grid.
        center (bool, optional): If True, the grid is centered at (0,0). Defaults to True.

    Returns:
        Point: The cell index as a point.
    """
    cell = Point(0, 0)
    cell.x += int(np.ceil((1.0 / cell_size) * p.x)) - 1
    cell.y += int(np.ceil((1.0 / cell_size) * p.y)) - 1
    if center:
        grid_center = grid_size // 2
        cell.x += grid_center
        cell.y += grid_center
    return cell


def get_point_from_idx(
    cell: Point, grid_size: int, cell_size: float, center: bool = True
) -> Point:
    """
    Get the coordinates of a point from its cell index in a grid.

    Args:
        cell (Point): The cell index.
        grid_size (int): The size of the grid.
        cell_size (float): The size of each cell in the grid.
        center (bool, optional): If True, the grid is centered at (0,0). Defaults to True.

    Returns:
        Point: The coordinates of the point.
    """
    p = Point(cell.x, cell.y)
    if center:
        grid_center = grid_size // 2
        p.x -= grid_center
        p.y -= grid_center
    p.x *= cell_size
    p.y *= cell_size
    return p


def cells_on_line(
    p1: Point, p2: Point, grid_size: int, cell_size: float, center: bool = True
) -> set:
    """
    Get the cells crossed by a line between two points in a grid.

    Args:
        p1 (Point): The first point.
        p2 (Point): The second point.
        grid_size (int): The size of the grid.
        cell_size (float): The size of each cell in the grid.
        center (bool, optional): If True, the grid is centered at (0,0). Defaults to True.

    Returns:
        set: A set of cell indices that the line crosses.
    """
    cells_crossed = set()
    if p1.x > p2.x:
        p1, p2 = p2, p1

    pcell = get_cell_idx(p1, grid_size, cell_size, center)
    dp = Point(p2.x - p1.x, p2.y - p1.y)

    step_x = 1 if dp.x > 0 else -1
    step_y = 1 if dp.y > 0 else -1

    tdelta_x = abs(cell_size / dp.x) if dp.x != 0 else np.inf
    tdelta_y = abs(cell_size / dp.y) if dp.y != 0 else np.inf

    if dp.x > 0:
        t_max_x = (
            tdelta_x * (1 - (p1.x % cell_size) / cell_size)
            if dp.x > 0
            else tdelta_x * ((p1.x % cell_size) / cell_size)
        )
        t_max_y = (
            tdelta_y * (1 - (p1.y % cell_size) / cell_size)
            if dp.y > 0
            else tdelta_y * ((p1.y % cell_size) / cell_size)
        )

    cells_crossed.add((pcell.x, pcell.y))

    while t_max_x < 1.0 or t_max_y < 1.0:
        if t_max_x < t_max_y:
            pcell.x += step_x
            t_max_x += tdelta_x
        else:
            pcell.y += step_y
            t_max_y += tdelta_y
        cells_crossed.add((pcell.x, pcell.y))
    return cells_crossed

def get_neighbors(y, x):
    """
    Get the neighboring coordinates of (x, y) on a 2D grid.
    """
    neighbors = [
        (y + 1, x),
        (y, x + 1),
        (y - 1, x),
        (y, x - 1),
        (y + 1, x + 1),
        (y + 1, x - 1),
        (y - 1, x + 1),
        (y - 1, x - 1),
    ]
    return neighbors