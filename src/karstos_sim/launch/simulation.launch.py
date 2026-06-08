import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_path = get_package_share_directory('karstos_sim')
    world_name = "simple_tunnel_02" # drone_world

    # Add the model path to the terminal
    models_path = os.path.join(pkg_path, 'models')
    set_gz_resource_path = SetEnvironmentVariable('ROS_GZ_SIM_MODEL_PATH', models_path)
    drone_model_path = os.path.join(models_path, 'drone', 'drone_model.sdf')
    world_path = os.path.join(pkg_path, 'worlds', f'{world_name}.sdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r ' + world_path}.items(),
    )

    spawn_drone = Node(
		package='ros_gz_sim',
		executable='create',
		arguments=[
			'-name', 'my_drone',
			'-file', drone_model_path,
			'-x', '0.0',
			'-y', '0.0',
			'-z', '1.0',
		]
	)

    # Remappings of the gazebo and ROS2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # IMU
            f'/world/{world_name}/model/my_drone/link/X3/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # GPS/NavSat
            f'/world/{world_name}/model/my_drone/link/X3/base_link/sensor/navsat/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            # LiDAR (Pointcloud and LaserScan)
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # Camera
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # Motors
            '/x3/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators'
        ],
        remappings=[
            (f'/world/{world_name}/model/my_drone/link/X3/base_link/sensor/imu_sensor/imu', '/drone/imu'),
            (f'/world/{world_name}/model/my_drone/link/X3/base_link/sensor/navsat/navsat', '/drone/gps'),
            ('/lidar', '/drone/lidar'),
            ('/lidar/points', '/drone/lidar_points'),
            ('/camera/image_raw', '/drone/camera'),
            ('/camera/camera_info', '/drone/camera_info'),
            ('/x3/command/motor_speed', '/drone/motors')
        ]
    )

    return LaunchDescription([
        set_gz_resource_path,
        gazebo,
        spawn_drone,
        bridge,
    ])