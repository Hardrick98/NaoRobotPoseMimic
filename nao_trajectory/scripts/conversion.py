import numpy as np
from scipy.spatial.transform import Rotation as R


def convert_to_quaternion(roll, pitch, yaw):
# Convert Euler angles to a rotation matrix
    rotation_matrix = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

    # Convert the rotation matrix to a quaternion
    quaternion = R.from_matrix(rotation_matrix).as_quat()

    print("Euler angles (roll, pitch, yaw):", np.array([roll, pitch, yaw]))
    print("Quaternion:", quaternion)
    return quaternion