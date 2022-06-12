from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hal_dc_motors',
            executable='hal_motor_driver'
        ),
        Node(
            package='hal_servo',
            executable='hal_servo_driver'
        ),
        Node(
            package='hal_encoders',
            executable='hal_encoders'
        ),
        Node(
            package='hal_ultrasonic_sensors',
            executable='hal_ultrasonic_sensor_driver'
        ),
        Node(
            package='drive_to_point',
            executable='main_driver'
        ),
        Node(
		    package='control',
		    executable='pid'
		)
    ])
