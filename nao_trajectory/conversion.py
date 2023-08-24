import numpy as np
from scipy.spatial.transform import Rotation as R

# Define the roll, pitch, and yaw angles (in radians)
roll = -0,266572
pitch = 0,744217
yaw = -0,805627 

# Convert Euler angles to a rotation matrix
rotation_matrix = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

# Convert the rotation matrix to a quaternion
quaternion = R.from_matrix(rotation_matrix).as_quat()

print("Euler angles (roll, pitch, yaw):", np.array([roll, pitch, yaw]))
print("Quaternion:", quaternion)