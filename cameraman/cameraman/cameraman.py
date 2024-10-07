#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseArray
from geometry_msgs.msg import Transform, TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from .TransformGenerator import TransformGenerator
from trajectory_msgs.msg import JointTrajectory
import argparse
import numpy as np
import time

class Cameraman(Node):

    def __init__(self, trajectory_name="vertical"):
        super().__init__("ar_cameraman")
        self.logger = self.get_logger()
        self.logger.info("Initializing Cameraman node")

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_publisher = self.create_publisher(TransformStamped, "/cameraman/tf", 10)
        self.trajectory_publisher = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)

        self.pose_array_publisher = self.create_publisher(PoseArray, "/cameraman/pose_array", 10)
       
        # Define the fixed target point the end-effector should focus on (focal point)
        self.target_point = Point()
        self.target_point.x = 0.0  # Adjust these coordinates as needed
        self.target_point.y = -0.36
        self.target_point.z = 0.05

        self.run_transforms(trajectory_name)
        time.sleep(2)
        exit()

    """
    TRANSFORM AND ROTATION FUNCTIONS
    """
    def run_transforms(self, trajectory_name):
        self.logger.info(f"Generating {trajectory_name} trajectory")
        gen = TransformGenerator(self.target_point)
        # First transform (given)
        object_transform = gen.create_transform(
            self.target_point.x,
            self.target_point.y,
            self.target_point.z
        )

        self.publish_transform(object_transform, frame_id="target_object")

        if trajectory_name == "vertical":
            step_offsets = self.get_trajectory_vertical()
        elif trajectory_name == "semi_circle":
            step_offsets = self.get_trajectory_semi_circle()
        elif trajectory_name == "circle":
            step_offsets = self.get_trajectory_circle()
        else:
            step_offsets = self.get_trajectory_horizontal()
        
        poses = []
        i = 0
        for offset in step_offsets:
            traj_transform = gen.create_pointed_transform(
                object_transform.translation.x + offset['x'],
                # object_transform.translation.y + offset['y'],
                offset['y'],
                offset['z'],
                object_transform
            )
            # self.publish_transform(traj_transform, frame_id=f"traj_{i}")
            poses.append(self.generate_pose(traj_transform))
            i += 1
 
        self.publish_pose_array(poses)
        self.logger.info("Poses generated and published")

    def get_trajectory_vertical(self):
        return [
            # X is offset from the target, y is offset from robot base
            {'x': 0.0, 'y': self.target_point.y + 0.2, 'z': 0.15},
            {'x': 0.0, 'y': self.target_point.y + 0.19, 'z': 0.16},
            {'x': 0.0, 'y': self.target_point.y + 0.15, 'z': 0.17},
            {'x': 0.0, 'y': self.target_point.y + 0.05, 'z': 0.18},
            {'x': 0.0, 'y': self.target_point.y, 'z': 0.19},
            {'x': 0.0, 'y': self.target_point.y, 'z': 0.2},
            {'x': 0.0, 'y': self.target_point.y + 0.05, 'z': 0.22},
            {'x': 0.0, 'y': self.target_point.y + 0.1, 'z': 0.25},
            {'x': 0.0, 'y': self.target_point.y + 0.15, 'z': 0.4},
            {'x': 0.0, 'y': self.target_point.y + 0.2, 'z': 0.5},
        ]

    def get_trajectory_horizontal(self):
        return [
            # X is offset from the target, y is offset from robot base
            {'x': -0.15, 'y': -0.5, 'z': 0.2},
            # {'x': -0.1125, 'y': -0.48, 'z': 0.13},
            {'x': -0.075, 'y': -0.46, 'z': 0.26},
            # {'x': -0.0375, 'y': -0.45, 'z': 0.18},
            {'x': 0.0, 'y': -0.45, 'z': 0.3},
            # {'x': 0.1125, 'y': -0.45, 'z': 0.18},
            {'x': 0.075, 'y': -0.46, 'z': 0.26},
            # {'x': 0.0375, 'y': -0.48, 'z': 0.13},
            {'x': 0.15, 'y': -0.5, 'z': 0.2},
        ]
    
    def get_trajectory_semi_circle(self):
        # Parameters for the semi-circle
        radius = 0.28
        num_points = 7  # Number of points in the trajectory
        y_base = -0.4   # Base y offset
        y_amplitude = -0.05  # Amplitude of y oscillation
        z_start = 0.2  # Starting z offset

        # Generate angles for the semi-circle (from 0 to π radians)
        angles = np.linspace(0, np.pi, num_points)

        # Calculate the x and z coordinates of the semi-circle
        x_coords = radius * np.cos(angles)
        z_coords = radius * np.sin(angles) + z_start

        # Create a slight oscillation for the y-coordinate
        y_coords = y_base + y_amplitude * np.sin(angles)

        # Create the trajectory list
        semi_circle_traj = [{'x': x, 'y': y, 'z': z} for x, y, z in zip(x_coords, y_coords, z_coords)]

        return reversed(semi_circle_traj)
    
    def get_trajectory_circle(self):
        parabola_width = 0.04
        radius = 0.08
        z_radius = 0.08
        z_value = 0.14
        num_points = 10
        angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)

        # Calculate the x and y coordinates of the circle
        # x_coords = radius * np.cos(angles) + 0.01
        x_coords = parabola_width * (angles - np.pi)**2 + -0.2
        y_coords = radius * np.sin(angles) + self.target_point.y
        z_coords = z_radius * np.cos(angles) + (z_radius * 2) + z_value

        # Create the trajectory list
        circle_traj = [{'x': x, 'y': y, 'z': z} for x, y, z in zip(x_coords, y_coords, z_coords)]

        circle_traj = circle_traj + [circle_traj[0]]

        return circle_traj


       
    def generate_pose(self, transform: Transform):
        pose = Pose()
        pose.position.x = transform.translation.x
        pose.position.y = transform.translation.y
        pose.position.z = transform.translation.z
        
        pose.orientation.w = transform.rotation.w
        pose.orientation.x = transform.rotation.x
        pose.orientation.y = transform.rotation.y
        pose.orientation.z = transform.rotation.z
        return pose
    
    def publish_transform(self, transform: Transform, frame_id = ""):
        try:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = frame_id
            t.transform = transform
            self.tf_static_broadcaster.sendTransform(t)
            self.tf_publisher.publish(t)
        except Exception as ex:
            self.get_logger().error(ex)
    
    def publish_pose_array(self, poses):
        try:
            pose_array = PoseArray()
            pose_array.header.stamp = self.get_clock().now().to_msg()
            pose_array.header.frame_id = 'base_link'
            pose_array.poses = poses
            self.pose_array_publisher.publish(pose_array)
        except Exception as ex:
            self.get_logger().error(ex)

def main(args=None):
    rclpy.init(args=args)

    # Argument parsing
    parser = argparse.ArgumentParser(description='Trajectory Name')
    parser.add_argument('--trajectory', type=str, help='Trajectory Name: vertical, horizontal')
    parsed_args = parser.parse_args()

    node = Cameraman(trajectory_name=parsed_args.trajectory)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {e}")
    finally:
        if rclpy.ok():
            node.get_logger().info("Shutting down rclpy")
            rclpy.shutdown()

if __name__ == "__main__":
    main()
