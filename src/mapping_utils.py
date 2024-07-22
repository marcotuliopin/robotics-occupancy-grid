from .grid_utils import cells_on_line
from .interfaces import Point


def get_perceptual_field(laser_points, robot_position, grid_size, cell_size):
    """
    Calculate the perceptual field of a robot in a grid based on laser points.

    Args: 
    laser_points (list): A list of tuples representing the x and y coordinates of the laser points. 
    robot_position (Point): The current position of the robot. 
    grid_size (int): The size of the grid. 
    cell_size (float): The size of each cell in the grid.

    Returns: set: A set of tuples representing the row and column indices of the cells in the perceptual field.
    """
    p_field = set()
    for i in range(len(laser_points)):
        ray_detection_global = Point(laser_points[i][0], laser_points[i][1])
        ray_path = cells_on_line(
            robot_position, ray_detection_global, grid_size, cell_size
        )

        for col, row in ray_path:
            if 0 <= col < grid_size and 0 <= row < grid_size:
                p_field.add((row, col))
    return p_field
