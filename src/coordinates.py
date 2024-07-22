import numpy as np

def homogeneous_transformation(origin, alpha, beta, gamma):
    """
    Calculates the homogeneous transformation matrix given the origin of
    the source coordinate system in relation to the destination coordinate system 
    and the euler angles.
    
    Parameters:
        -origin (1d np.ndarray): origin of the source coordinate system in relation
            to the destination coordinate system.
        -alpha (float): rotation angle in Z.
        -beta (float): rotation angle in Y.
        -gamma (float): rotation angle in X.
    
    Returns:
        -T (2d np.ndarray): homogeneous transformation matrix.
    """
    R = rotation_matrix(alpha, axis=0) @ rotation_matrix(beta, axis=1) @ rotation_matrix(gamma, axis=2)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = origin
    
    return T

def rotation_matrix(theta, axis=2):
    """
    Calculates the rotation matrix of a rigid transformation with a rotation 
    theta in a given axis.

    Parameters
        -theta (float): rotation angle.
        -axis (int): axis in which the rotation happen.
        
    Returns
        -R (2d np.ndarray): rotation matrix.
    """
    if axis == 0:
        # Rotation in the X-axis
        return np.array([[ 1,              0,             0 ],
                         [ 0, np.cos(theta), -np.sin(theta) ],
                         [ 0, np.sin(theta),  np.cos(theta) ]])
    elif axis == 1:
        # Rotation in the Y-axis
        return np.array([[ np.cos(theta),  0, -np.sin(theta)],
                         [ 0,              1,             0 ],
                         [ np.sin(theta),  0, np.cos(theta) ]])
    else:
        # Rotation in the Z-axis
        return np.array([[ np.cos(theta), -np.sin(theta), 0 ],
                         [ np.sin(theta),  np.cos(theta), 0 ],
                         [ 0            ,  0            , 1 ]])

def calculate_object_transformation(objHandle, refHandle, sim):
    """
    Returns the homogeneous transformation matrix of an object given its handle
    and the reference handle.
    
    Parameters:
        -objHandle (int): handle of the target object.
        -refHandle (int): handle of the object to use as reference in the transformation.
    
    Returns:
        -T (2d np.ndarray): transformation matrix of the target object.        
    """
    position = sim.getObjectPosition(objHandle, refHandle)
    orientation = sim.getObjectOrientation(objHandle, refHandle)
    return homogeneous_transformation(position, *orientation)

def global_references(paths, sim):
    """
    Get the position and orientation of objects in relation to the global coordinates.
    
    Parameters:
        -paths (list): paths to the objects of the scene.
    Returns:
        -global_refs (list): homogeneous transformation matrixes of the points.
    """
    global_refs = []
    for path in paths:
        handle = sim.getObject(path)
        T = calculate_object_transformation(handle, sim.handle_world, sim)
        global_refs.append(T)
    
    return global_refs

def transform_coordinates(origin, *args):
    """
    Transform a set of points from one coordinate system to another.
    
    Parameters:
        -origin (2d np.ndarray): homogeneous transformation matrix of the point to be
            the origin of the coordinate system.
        -*args (list): points to be transformed.
    Returns:
        -new_origin (2d np.ndarray): transformation matrix from the old coordinate system
            to the new one.
        -local_refs (list): points in the new coordinate system.
    """
    new_origin = np.eye(4)
    new_origin[:3, :3] = origin[:3, :3].T
    new_origin[:3, 3] = -origin[:3, :3].T @ origin[:3, 3]
    
    local_refs =[]
    for arg in args:
        local_refs.append(new_origin @ arg)
    
    return new_origin, local_refs