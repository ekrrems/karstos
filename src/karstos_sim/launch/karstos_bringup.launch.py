import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, TimerAction, ExecuteProcess, IncludeLaunchDescription

def generate_launch_description():
    
    px4_dir = os.path.expanduser("~/PX4-Autopilot")
    
    # Run microAgent
    microAgent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        output="screen"
    )

    px4 = ExecuteProcess(
        cmd=[
            "bash", "-c",
            f"cd {px4_dir} && PX4_GZ_WORLD=simple_tunnel_02 make px4_sitl gz_x500"
        ],
        output="screen"
    )

    offboard = Node(
        package="karstos_control",
        executable="offboard_controller",
        output="screen"
    )

    # Vision / noisy pose → PX4 EKF
    vision = Node(
        package="karstos_control",
        executable="vision_pose_publisher",
        output="screen"
    )

    return LaunchDescription([
        microAgent,
        px4,
        # vision,
        # offboard
    ])