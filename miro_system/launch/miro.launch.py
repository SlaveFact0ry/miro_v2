import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'miro_system'
    
    # 1. TF: Base -> Lidar (Pitch 10도 = 0.1745 rad)
    # 위치: 로봇 중심에서 앞(x) 0.15m, 높이(z) 0.1m 가정 (실측 후 수정 필요)
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar',
        arguments=['0.15', '0', '0.1', '0', '0.1745', '0', 'base_link', 'laser_frame']
    )

    # 2. TF: Base -> IMU (중심점)
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']
    )

    # 3. TF: Base -> Sonar Sensors (초음파 위치 정의 - 시각화용)
    # 예: 왼쪽 30도, 오른쪽 -30도
    sonar_l_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_sonar_l',
        arguments=['0.1', '0.1', '0.05', '0.52', '0', '0', 'base_link', 'sonar_left_link']
    )
    # (필요시 Center, Right도 추가)

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

    return LaunchDescription([
        lidar_tf,
        imu_tf,
        sonar_l_tf,
        rplidar_node,
        micro_ros_agent
        # EKF 노드는 센서 퓨전 튜닝할 때 켜겠습니다. 지금은 주석 처리 또는 생략
    ])
