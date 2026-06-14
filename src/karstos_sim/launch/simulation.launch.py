import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, TimerAction, ExecuteProcess

# def generate_launch_description():

#     pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
#     pkg_path       = get_package_share_directory('karstos_sim')
#     pkg_slam       = get_package_share_directory('slam_toolbox')

#     world_name       = 'simple_tunnel_02'
#     models_path      = os.path.join(pkg_path, 'models')
#     drone_model_path = os.path.join(models_path, 'drone', 'drone_model.sdf')
#     world_path       = os.path.join(pkg_path, 'worlds', f'{world_name}.sdf')
#     slam_config      = os.path.join(pkg_path, 'config', 'slam.yaml')

#     set_gz_resource_path = SetEnvironmentVariable(
#         'GZ_SIM_RESOURCE_PATH', models_path
#     )

#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
#         ),
#         launch_arguments=[('gz_args', '-r ' + world_path)],
#     )

#     spawn_drone = Node(
#         package='ros_gz_sim',
#         executable='create',
#         arguments=[
#             '-name', 'my_drone',
#             '-file', drone_model_path,
#             '-x', '0.0', '-y', '0.0', '-z', '1.0',
#         ],
#         output='screen',
#     )

#     # bridge = TimerAction(period=2.0, actions=[ Node(
#     #     package='ros_gz_bridge',
#     #     executable='parameter_bridge',
#     #     arguments=[
#     #         f'/world/{world_name}/model/my_drone/link/X3/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
#     #         f'/world/{world_name}/model/my_drone/link/X3/base_link/sensor/navsat/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
#     #         '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
#     #         '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
#     #         '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
#     #         '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
#     #         '/x3/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators',
#     #         '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
#     #         # f'/world/{world_name}/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
#     #     ],
#     #     remappings=[
#     #         (f'/world/{world_name}/model/my_drone/link/X3/base_link/sensor/imu_sensor/imu', '/drone/imu'),
#     #         (f'/world/{world_name}/model/my_drone/link/X3/base_link/sensor/navsat/navsat', '/drone/gps'),
#     #         ('/lidar',              '/drone/lidar'),
#     #         ('/lidar/points',       '/drone/lidar_points'),
#     #         ('/camera/image_raw',   '/drone/camera'),
#     #         ('/camera/camera_info', '/drone/camera_info'),
#     #         ('/x3/command/motor_speed', '/drone/motors'),
#     #         # (f'/world/{world_name}/pose/info', '/gazebo_poses'),
#     #     ],
#     #     output='screen',
#     # )])
#     bridge = Node(
#         package="ros_gz_bridge",
#         executable="parameter_bridge",
#         arguments=[
#             #IMU
#             f"/world/{world_name}/model/my_drone/link/X3/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
#             # Lidar (Scan)
#             f"/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
#             # LiDAR (cloud)
#             '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
#             # Camera
#             '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
#             '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
#             # Motors (ROS → Gazebo)
#             '/x3/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators',
#             # Clock (optional)
#             '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
#             # Drone pose and TF
#             '/model/my_drone/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
#         ],
#         remappings=[
#             # IMU
#             (f"/world/{world_name}/model/my_drone/link/X3/base_link/sensor/imu_sensor/imu", "/drone/imu"),
#             # LiDAR
#             ('/lidar', '/drone/lidar'),
#             ('/lidar/points', '/drone/lidar_points'),
#             # Camera
#             ('/camera/image_raw', '/drone/camera'),
#             ('/camera/camera_info', '/drone/camera_info'),
#             # Motor
#             ('/x3/command/motor_speed', '/drone/motors'),
#             # Drone pose
#             ('/model/my_drone/pose', '/drone/pose_raw'),
#         ],
#         output='screen',
#     )

#     # NO use_sim_time on static TFs — they are valid for all time
#     tf_map_odom = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
#         parameters=[{'use_sim_time': True}],
#     )

#     # tf_odom_lidar = Node(
#     #     package='tf2_ros',
#     #     executable='static_transform_publisher',
#     #     arguments=[
#     #         '0', '0', '0', '0', '0', '0',
#     #         'odom',
#     #         'my_drone/X3/base_link/drone_lidar',
#     #     ],
#     # )

#     tf_odom_base = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         arguments=['0','0','0','0','0','0',
#                 'odom','my_drone/X3/base_link'],
#         parameters=[{'use_sim_time': True}],
#     )

#     # base_link → drone_lidar (sensor is 10cm above body)
#     tf_base_lidar = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         arguments=['0','0','0.1','0','0','0',
#                 'my_drone',
#                 'my_drone/X3/base_link/drone_lidar'],
#         parameters=[{'use_sim_time': True}],
#     )

#     pose = TimerAction(
#         period=3.0,
#         actions=[
#             Node(
#                 package='karstos_control',
#                 executable='pose_extractor',
#                 output='screen',
#             )
#         ]
#     )

#     slam = TimerAction(
#         period=10.0,
#         actions=[
#             IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource(
#                     os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
#                 ),
#                 launch_arguments=[
#                     ('slam_params_file', slam_config),
#                     ('use_sim_time', 'true'),
#                 ],
#             )
#         ],
#     )

#     controller = TimerAction(
#         period=10.0,
#         actions=[
#             Node(
#                 package='karstos_control',
#                 executable='flight_controller',
#                 name='flight_controller',
#                 output='screen',
#                 parameters=[{'use_sim_time': True}],
#             )
#         ],
#     )

#     rviz = TimerAction(
#         period=8.0,
#         actions=[
#             Node(
#                 package='rviz2',
#                 executable='rviz2',
#                 name='rviz2',
#                 output='screen',
#             )
#         ],
#     )

#     return LaunchDescription([
#         set_gz_resource_path,
#         gazebo,
#         spawn_drone,
#         bridge,
#         # tf_map_odom, # and this
#         # tf_odom_base,
#         tf_base_lidar, # This 
#         # slam,
#         # pose,
#         controller,
#         rviz,
#     ])


#     ##################################################

# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import ExecuteProcess


def generate_launch_description():

    # PX4 SITL
    px4 = ExecuteProcess(
        cmd=["bash", "-c", "cd ~/PX4-Autopilot && make px4_sitl gz_x500"],
        output="screen"
    )

    # MicroXRCEAgent (DDS bridge)
    xrce = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        output="screen"
    )

    bridge = TimerAction(period=2.0, actions=[ Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            "/drone/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image"
        ],
        # remappings=[
        #     (f'/world/{world_name}/model/my_drone/link/X3/base_link/sensor/imu_sensor/imu', '/drone/imu'),
        #     (f'/world/{world_name}/model/my_drone/link/X3/base_link/sensor/navsat/navsat', '/drone/gps'),
        #     ('/lidar',              '/drone/lidar'),
        #     ('/lidar/points',       '/drone/lidar_points'),
        #     ('/camera/image_raw',   '/drone/camera'),
        #     ('/camera/camera_info', '/drone/camera_info'),
        #     ('/x3/command/motor_speed', '/drone/motors'),
        #     # (f'/world/{world_name}/pose/info', '/gazebo_poses'),
        # ],
        output='screen',
    )])

    # Offboard controller (your setpoints)
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

    tf_node = Node(
        package="karstos_control",
        executable="px4_tf_broadcaster",
        output="screen"
    )

    return LaunchDescription([
        # px4,
        # xrce,
        vision,
        offboard,
        tf_node
    ])