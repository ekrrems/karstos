import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, TimerAction, ExecuteProcess


# def generate_launch_description():

#     # PX4 SITL
#     px4 = ExecuteProcess(
#         cmd=["bash", "-c", "cd ~/PX4-Autopilot && make px4_sitl gz_x500"],
#         output="screen"
#     )

#     # MicroXRCEAgent (DDS bridge)
#     xrce = ExecuteProcess(
#         cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
#         output="screen"
#     )

#     bridge = TimerAction(period=2.0, actions=[ Node(
#         package='ros_gz_bridge',
#         executable='parameter_bridge',
#         arguments=[
#             '/lidar_3d/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
#             '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image',
#             '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
#             '/world/simple_tunnel_02/model/x500_vision_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image',
#             '/world/simple_tunnel_02/model/x500_vision_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
#             '/world/simple_tunnel_02/model/x500_vision_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
#         ],
#         remappings=[
#             ("/lidar_3d/points", "/sensors/lidar/points"),
#             ("/depth_camera", "/camera/depth/image_raw"),
#             ("/camera_info", "/camera/depth/camera_info"),
#             ("/world/simple_tunnel_02/model/x500_vision_0/link/camera_link/sensor/IMX214/image", "/camera/rgb/image_raw"),
#             ("/world/simple_tunnel_02/model/x500_vision_0/link/camera_link/sensor/IMX214/camera_info", "/camera/rgb/camera_info"),
#             ("/world/simple_tunnel_02/model/x500_vision_0/link/base_link/sensor/imu_sensor/imu", "/sensors/imu/data"),
#         ],
#         output='screen',
#     )])

#     # Offboard controller (your setpoints)
#     offboard = Node(
#         package="karstos_control",
#         executable="offboard_controller",
#         output="screen"
#     )

#     # Vision / noisy pose → PX4 EKF
#     vision = Node(
#         package="karstos_control",
#         executable="vision_pose_publisher",
#         output="screen"
#     )

#     tf_node = Node(
#         package="karstos_control",
#         executable="px4_tf_broadcaster",
#         output="screen"
#     )

#     return LaunchDescription([
#         # px4,
#         # xrce,
#         bridge,
#         vision,
#         offboard,
#         tf_node
#     ])

from launch import LaunchDescription
from launch_ros.actions import Node


def make_bridge(
    name: str,
    bridge_argument: str,
    gz_topic: str,
    ros_topic: str,
) -> Node:
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=name,
        output="screen",
        arguments=[bridge_argument],
        remappings=[
            (gz_topic, ros_topic),
        ],
    )


def generate_launch_description():

    # ---------------------------------------------------------
    # 3D LiDAR PointCloud2
    # ---------------------------------------------------------
    lidar_points_bridge = make_bridge(
        name="lidar_points_bridge",
        bridge_argument=(
            "/lidar_3d/points"
            "@sensor_msgs/msg/PointCloud2"
            "[gz.msgs.PointCloudPacked"
        ),
        gz_topic="/lidar_3d/points",
        ros_topic="/sensors/lidar/points",
    )

    # ---------------------------------------------------------
    # Depth image
    # ---------------------------------------------------------
    depth_image_bridge = make_bridge(
        name="depth_image_bridge",
        bridge_argument=(
            "/depth_camera"
            "@sensor_msgs/msg/Image"
            "[gz.msgs.Image"
        ),
        gz_topic="/depth_camera",
        ros_topic="/camera/depth/image_raw",
    )

    # ---------------------------------------------------------
    # Depth-camera 3D point cloud
    # This entry was missing in your previous launch file.
    # ---------------------------------------------------------
    depth_points_bridge = make_bridge(
        name="depth_points_bridge",
        bridge_argument=(
            "/depth_camera/points"
            "@sensor_msgs/msg/PointCloud2"
            "[gz.msgs.PointCloudPacked"
        ),
        gz_topic="/depth_camera/points",
        ros_topic="/camera/depth/points",
    )

    # ---------------------------------------------------------
    # Depth camera calibration
    # ---------------------------------------------------------
    depth_info_bridge = make_bridge(
        name="depth_info_bridge",
        bridge_argument=(
            "/camera_info"
            "@sensor_msgs/msg/CameraInfo"
            "[gz.msgs.CameraInfo"
        ),
        gz_topic="/camera_info",
        ros_topic="/camera/depth/camera_info",
    )

    # ---------------------------------------------------------
    # RGB camera
    # ---------------------------------------------------------
    rgb_image_gz_topic = (
        "/world/simple_tunnel_02/model/x500_vision_0/"
        "link/camera_link/sensor/IMX214/image"
    )

    rgb_image_bridge = make_bridge(
        name="rgb_image_bridge",
        bridge_argument=(
            rgb_image_gz_topic
            + "@sensor_msgs/msg/Image"
            + "[gz.msgs.Image"
        ),
        gz_topic=rgb_image_gz_topic,
        ros_topic="/camera/rgb/image_raw",
    )

    rgb_info_gz_topic = (
        "/world/simple_tunnel_02/model/x500_vision_0/"
        "link/camera_link/sensor/IMX214/camera_info"
    )

    rgb_info_bridge = make_bridge(
        name="rgb_info_bridge",
        bridge_argument=(
            rgb_info_gz_topic
            + "@sensor_msgs/msg/CameraInfo"
            + "[gz.msgs.CameraInfo"
        ),
        gz_topic=rgb_info_gz_topic,
        ros_topic="/camera/rgb/camera_info",
    )

    # ---------------------------------------------------------
    # Raw Gazebo IMU
    # ---------------------------------------------------------
    imu_gz_topic = (
        "/world/simple_tunnel_02/model/x500_vision_0/"
        "link/base_link/sensor/imu_sensor/imu"
    )

    imu_bridge = make_bridge(
        name="imu_bridge",
        bridge_argument=(
            imu_gz_topic
            + "@sensor_msgs/msg/Imu"
            + "[gz.msgs.IMU"
        ),
        gz_topic=imu_gz_topic,
        ros_topic="/sensors/imu/data_raw",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],
        output="screen",
    )

    # ---------------------------------------------------------
    # Your existing ROS nodes
    # ---------------------------------------------------------
    offboard = Node(
        package="karstos_control",
        executable="offboard_controller",
        name="offboard_controller",
        output="screen",
    )

    vision = Node(
        package="karstos_control",
        executable="vision_pose_publisher",
        name="vision_pose_publisher",
        output="screen",
    )

    tf_node = Node(
        package="karstos_control",
        executable="px4_tf_broadcaster",
        name="px4_tf_broadcaster",
        parameters=[
            {"use_sim_time": True}
        ],
        output="screen",
    )

    tf_base_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_camera",
        arguments=[
            "--x", "0.12",
            "--y", "0.03",
            "--z", "0.242",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "camera_link",
        ],
    )

    tf_base_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_lidar",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.32",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "lidar_3d_link",
        ],
    )

    sensor_bridges = TimerAction(
        period=2.0,
        actions=[
            lidar_points_bridge,
            depth_image_bridge,
            depth_points_bridge,
            depth_info_bridge,
            rgb_image_bridge,
            rgb_info_bridge,
            imu_bridge,
        ],
    )

    rviz = TimerAction(
        period=5.0,
        actions=[
             Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                output="screen",
                parameters=[
                    {"use_sim_time": True}
                ],
            )
        ]
    )

    return LaunchDescription([
        clock_bridge,
        # Dynamic transform:
        # odom -> base_link
        tf_node,

        # Static sensor transforms:
        # base_link -> camera_link
        # base_link -> lidar_3d_link
        tf_base_camera,
        tf_base_lidar,

        # Start sensor bridges after two seconds
        sensor_bridges,

        vision,
        offboard,

        rviz
    ])