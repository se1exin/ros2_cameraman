import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def load_yaml(package_name, file_name):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_name)
    with open(absolute_file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="False")
    rviz_config_file = LaunchConfiguration("rviz_config_file", default="moveit.rviz")

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Make MoveIt use simulation time. This is needed "+\
                "for trajectory planing in simulation.",
        ))

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="moveit.rviz",
            description="Full path to the RViz configuration file to use",
        ))

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare("ar_description"), "urdf", "ar.urdf.xacro"]),
        " ",
        "name:=ar",
        " ",
        "include_gripper:=",
        "False",
    ])
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare("ar_moveit_config"), "srdf", "ar.srdf.xacro"]),
        " ",
        "name:=ar",
        " ",
        "include_gripper:=",
        "False",
    ])
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    robot_description_kinematics = {
        "robot_description_kinematics":
        load_yaml(
            "cameraman_moveit",
            os.path.join("config", "kinematics.yaml"),
        )
    }

    robot_description_planning = {
        "robot_description_planning":
        load_yaml(
            "cameraman_moveit",
            os.path.join("config", "joint_limits.yaml"),
        )
    }

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
        }
    }
    ompl_planning_yaml = load_yaml("cameraman_moveit",
                                   "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    controllers_yaml = load_yaml("cameraman_moveit", "config/controllers.yaml")

    moveit_params = load_yaml("cameraman_moveit", "config/moveit_parameters.yaml")

    moveit_controllers = {
        "moveit_simple_controller_manager":
        controllers_yaml,
        "moveit_controller_manager":
        "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution": {
            "allowed_execution_duration_scaling": 1.2,
            "allowed_goal_duration_margin": 0.5,
            "allowed_start_tolerance": 20.0,
        }
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    params_dict = {}
    params_dict.update(robot_description)
    params_dict.update(robot_description_semantic)
    params_dict.update(robot_description_planning)
    params_dict.update(robot_description_kinematics)
    params_dict.update(ompl_planning_pipeline_config)
    params_dict.update(moveit_params)
        
    params_dict.update(moveit_controllers)
    params_dict.update(trajectory_execution)
    params_dict.update(planning_scene_monitor_parameters)
    params_dict.update(
        {
            "use_sim_time": use_sim_time
        }
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            moveit_params,
            {
                "use_sim_time": use_sim_time
            },
        ],
    )

    moveit_node = Node(
        package="cameraman_moveit",
        executable="cameraman_moveit",
        output="screen",
        parameters=[params_dict],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("cameraman_moveit"), "config", rviz_config_file])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
        ],
    )

    return LaunchDescription([
      rviz_node,
      move_group_node,
      moveit_node
    ])
