import numpy as np

from .constants import *
from .coordinates import calculate_object_transformation
from .sensor_utils import handle_laser_data, read_sensor_data

# (y, x)

def walk(pcurr, ocurr, pgoal, sim, rmotor, lmotor, sensor_handle):
    """
    Function to perform a walk in potential fields.

    Args:
        pcurr (np.array): Current position of the robot.
        ocurr (float): Current orientation of the robot.
        pgoal (np.array): Goal position.
        sim (object): Simulation object.
        rmotor (int): Handle for right motor.
        lmotor (int): Handle for left motor.
        sensor_handle (int): Handle for sensor.

    Returns:
        None
    """
    err = pgoal - pcurr
    fattr = err / np.linalg.norm(err)  # Attraction Force

    # Get sensor data
    sensor_range_data, sensor_angle_data = read_sensor_data(sim)
    while sensor_range_data == -1:
        sensor_range_data, sensor_angle_data = read_sensor_data(sim)
    sensor_data = np.array([sensor_angle_data, sensor_range_data]).T

    # Get the position and orientation of the laser in relation to the global coordinates
    W_LT = calculate_object_transformation(sensor_handle, sim.handle_world, sim)
    global_laser_points, _ = handle_laser_data(sensor_data, W_LT)

    # Repulsion Force
    frep = 0
    for i in range(len(global_laser_points)):
        _, dist = sensor_data[i]
        if dist < RANGE:
            # Calculate the repulsion force of the obstacle
            dv = pcurr - global_laser_points[i][:2]
            d = np.linalg.norm(dv)
            frep += KREP * (1 / d**2) * ((1 / d) - (1 / RANGE)) * (dv / d)

    # Total force
    ft = fattr  + frep

    fx = ft[0]
    fy = ft[1]

    # Control [De Luca e Oriolo, 1994]
    v = KR * (fx * np.cos(ocurr) + fy * np.sin(ocurr))
    w = KT * (np.arctan2(fy, fx) - ocurr)

    v = max(min(v, MAXV), -MAXV)
    w = max(min(w, MAXW), -MAXW)

    vr = ((2.0 * v) + (w * L)) / (2.0 * R)
    vl = ((2.0 * v) - (w * L)) / (2.0 * R)
    set_velocity(sim, rmotor, vr, lmotor, vl)


def set_velocity(sim, rmotor, vr, lmotor, vl):
    sim.setJointTargetVelocity(rmotor, vr)
    sim.setJointTargetVelocity(lmotor, vl)