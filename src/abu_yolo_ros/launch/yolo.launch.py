from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare("abu_yolo_ros"),
        "config",
        "yolo_detection.yaml"
    ])

    return LaunchDescription([
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="camera",
            output="screen",
            parameters=[
                {
                    "video_device": "/dev/video0"
                }
            ]
        ),

        Node(
            package="abu_yolo_ros",
            executable="yolo_detection_node",
            name="yolo_detection_node",
            output="screen",
            parameters=[config_file]
        )
    ])