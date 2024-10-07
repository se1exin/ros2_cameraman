import numpy as np
from geometry_msgs.msg import Transform
from tf_transformations import quaternion_from_matrix, quaternion_multiply, quaternion_about_axis

class TransformGenerator:
    def __init__(self, target_point):
        """
        Initialize the TransformGenerator with the target point, which represents the first transform's position.
        :param target_point: A point object with x, y, z attributes (usually the first transform's position).
        """
        self.target_point = target_point

    def create_transform(self, x, y, z):
        """
        Create a basic transform with given x, y, z translation.
        :param x: Translation along X axis.
        :param y: Translation along Y axis.
        :param z: Translation along Z axis.
        :return: Transform object with the specified translation.
        """
        transform = Transform()
        transform.translation.x = x
        transform.translation.y = y
        transform.translation.z = z
        return transform
    
    def create_pointed_transform(self, x, y, z, target_transform):
        transform = self.create_transform(x, y , z)
        return self.point_at_transform(target_transform, transform)

    def point_at_transform(self, target_transform, other_transform):
        
      # Vector from third transform to first transform
      direction_vector_third = np.array([
          target_transform.translation.x - other_transform.translation.x,
          target_transform.translation.y - other_transform.translation.y,
          target_transform.translation.z - other_transform.translation.z
      ])

      # Normalize the direction vector
      direction_vector_third = direction_vector_third / np.linalg.norm(direction_vector_third)

      # Basis vectors for the third transform
      z_axis_third = direction_vector_third  # Z-axis should point towards the first transform
      x_axis_third = np.array([1, 0, 0])  # Assuming X-axis stays aligned with the world frame
      y_axis_third = np.cross(z_axis_third, x_axis_third)  # Y-axis is perpendicular to X and Z

      # Recompute X-axis to ensure orthogonality
      x_axis_third = np.cross(y_axis_third, z_axis_third)

      # Create the rotation matrix from these axes
      rotation_matrix_third = np.array([
          [x_axis_third[0], y_axis_third[0], z_axis_third[0], 0],
          [x_axis_third[1], y_axis_third[1], z_axis_third[1], 0],
          [x_axis_third[2], y_axis_third[2], z_axis_third[2], 0],
          [0,        0,        0,        1]
      ])

      # Convert the rotation matrix to a quaternion
      quaternion_third = quaternion_from_matrix(rotation_matrix_third)

      # Apply a 180-degree rotation around the Z-axis to flip the Y-axis downward
      rotation_180_z_third = quaternion_about_axis(np.pi, [0, 0, 1])  # 180-degree rotation around Z-axis

      # Combine the two quaternions (first the one that aligns Z-axis, then the 180-degree Z rotation)
      final_quaternion_third = quaternion_multiply(quaternion_third, rotation_180_z_third)

      # Apply the final quaternion to the third transform
      other_transform.rotation.x = final_quaternion_third[0]
      other_transform.rotation.y = final_quaternion_third[1]
      other_transform.rotation.z = final_quaternion_third[2]
      other_transform.rotation.w = final_quaternion_third[3]

      return other_transform
