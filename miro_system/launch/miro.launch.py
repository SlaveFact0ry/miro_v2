import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'miro_system'
    ekf_config_path = os.path.join(get_package_share_directory(pkg_name), 'config', 'ekf.yaml')
    # 1. TF: Base -> Lidar (Pitch 10도 = 0.1745 rad)
    # 위치: 로봇 중심에서 앞(x) 0.15m, 높이(z) 0.1m 가정 (실측 후 수정 필요)
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar',
        arguments=['0.15', '0', '0.1', '0', '0.1745', '0', 'base_link', 'laser_frame']
    )

    # 2. TF: Base -> IMU 
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']
    )

    # 3. TF: Base -> Sonar Sensors 
    
    sonar_l_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_sonar_l',
        arguments=['0.1', '0.1', '0.05', '0.52', '0', '0', 'base_link', 'sonar_left_link']
    )
   

    # 4. RPLIDAR Driver
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }]
    )

    # 5. Micro-ROS Agent (ESP32)
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=["serial", "--dev", "/dev/ttyACM0", "-b", "115200"], # [확인됨]
        output='screen'
    )
    # 6. RF2O Laser Odometry (Lidar -> Odom)
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',       # EKF로 들어갈 토픽 이름
            'publish_tf': False,              # TF는 EKF가 할 것이므로 끈다!
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0                      # Lidar 주사율(10Hz)에 맞춤
        }]
    )

    # 7. EKF Node ( Localization)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[('odometry/filtered', 'odom')] # 최종 출력 토픽
    )
    return LaunchDescription([
        lidar_tf,
        imu_tf,
        sonar_l_tf,
        rplidar_node,
        micro_ros_agent,
        rf2o_node,
        ekf_node
    ])
