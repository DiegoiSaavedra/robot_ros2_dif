from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Lanzador principal para el robot diferencial.  Este archivo inicia el nodo
    `base_driver`, el driver del LIDAR y el nodo del IMU.  Ajusta los parámetros
    según tu configuración de hardware.
    """
    ld = []

    # Nodo base_driver (odometría y comando de motores)
    ld.append(
        Node(
            package='robot_difer',
            executable='base_driver',
            name='base_driver',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baud': 115200,
                # Parámetros físicos del robot
                'wheel_radius': 0.035,
                'ticks_per_revolution': 4532,
                'base_width': 0.145,
            }]
        )
    )

    # Driver LIDAR (Youyeetoo L90D) usando ydlidar_ros2_driver
    ld.append(
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB1',
                'frame_id': 'laser_frame',
                'baudrate': 230400,
                'angle_min': -180.0,
                'angle_max': 180.0,
                'range_min': 0.01,
                'range_max': 64.0,
                'frequency': 10.0,
                # Otros parámetros según documentación del driver
            }]
        )
    )

    # Nodo IMU MPU9250 (publica sensor_msgs/Imu en /imu)
    ld.append(
        Node(
            package='mpu9250',
            executable='mpu9250_node',
            name='mpu9250',
            output='screen',
            parameters=[{
                'frame_id': 'imu_link',
                # Otros parámetros de calibración si fuese necesario
            }]
        )
    )

    return LaunchDescription(ld)
