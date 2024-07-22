import numpy as np

# Extracted from github: https://github.com/verlab/dcc042-robotica-movel/blob/main/jupyter-notebooks/aula03-ferramental.ipynb
def read_sensor_data(sim, range_data_signal_id="hokuyo_range_data", 
                   angle_data_signal_id="hokuyo_angle_data"):
    """
    It will try to capture the range and angle data from the simulator.
    The request for the range data is sent in streaming mode to force
    it to sync with the angle data request which acts as a mutex.

    Parameters:
        -range_data_signal_id: string containing the range data signal pipe name.
        -angle_data_signal_id: string containing the angle data signal pipe name.
    
    Returns:
        -returns None if no data is recovered.
        -returns two arrays, one with data range and the other with their angles, if data was 
            retrieved successfully.
    """
    # The first call should be non-blocking to avoid getting out-of-sync angle data
    string_range_data = sim.getStringSignal(range_data_signal_id)

    # The second call should block to avoid out-of-sync scenarios
    # between your python script and the simulator's main loop
    # (your script may be slower than the simulator's main loop, thus
    # slowing down data processing)
    string_angle_data = sim.waitForSignal(angle_data_signal_id)

    # Check the if both data were obtained correctly
    if string_range_data and string_angle_data:
        # Unpack data from range and sensor messages
        raw_range_data = sim.unpackFloatTable(string_range_data)
        raw_angle_data = sim.unpackFloatTable(string_angle_data)

        return raw_range_data, raw_angle_data

    # Return none in case were nothing was gotten from the simulator
    return -1, -1

def handle_laser_data(laser_data, W_LT, max_sensor_range=-1.):
    """
    This function takes laser data and transforms it from the laser
    reference frame to the global reference frame using provided
    transformation matrices. It calculates the detection points of
    the laser in both the local and global reference frames.

    Parameters:
        - laser_data (list): a list of tuples containing the
            angle and distance of laser detections.
        - W_LT (numpy.ndarray): transformation matrix for converting
            laser points from the local reference frame to the global
            reference frame.
        - max_sensor_range (float, optional): maximum sensor range for
            filtering laser detections. Defaults to 5 units.

    Returns:
        - global_laser_points (np.ndarray): an array of shape containing
            the laser detection points in the global reference frame.
        - local_laser_points (np.ndarray): an array of shape containing
            the laser detection points in the local reference frame.

    """
    # Calculate laser detection points in the laser reference frame
    if max_sensor_range < 0:
        local_laser_points = np.ones((len(laser_data), 4))
        for i in range(len(laser_data)):
            ang, dist = laser_data[i]
            local_laser_points[i, 0] = dist * np.cos(ang)
            local_laser_points[i, 1] = dist * np.sin(ang)
            local_laser_points[i, 2] = 0
    else:
        local_laser_points = np.ones((len(laser_data), 4))
        for i in range(len(laser_data)):
            ang, dist = laser_data[i]
            local_laser_points[i, 0] = max_sensor_range * np.cos(ang)
            local_laser_points[i, 1] = max_sensor_range * np.sin(ang)
            local_laser_points[i, 2] = 0
        
    # Calculate laser detection points in the global reference frame
    global_laser_points = np.dot(W_LT, local_laser_points.T).T

    return global_laser_points, local_laser_points

def get_sensor_data(sim):
    sensor_range_data, sensor_angle_data = read_sensor_data(sim)
    while sensor_range_data == -1:
        sensor_range_data, sensor_angle_data = read_sensor_data(sim)
    sensor_data = np.array([sensor_angle_data, sensor_range_data]).T
    return sensor_data