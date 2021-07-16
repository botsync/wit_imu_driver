#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wit_imu_driver',
            executable='wit_imu_driver',
            name='wit_imu_driver',
            output="screen",
            emulate_tty=True,
            parameters=[
            	{"device" : "/dev/imu"},
            	{"frame_id" : "imu"},
                {'baud': 115200}],
            remappings=[
                ('data_raw', 'imu/data')
            ]
        )
    ])






