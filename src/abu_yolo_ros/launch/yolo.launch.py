from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare("abu_yolo_ros"),
        "config",
        "yolo_detection.yaml"
    ])
    inference_backend = LaunchConfiguration("inference_backend")

    return LaunchDescription([
        DeclareLaunchArgument(
            "inference_backend",
            default_value="onnxruntime",
            description="Inference backend for yolo_detection_node: onnxruntime or tensorrt",
        ),

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
            parameters=[
                config_file,
                {
                    "inference.backend": inference_backend
                }
            ]
        ),

        Node(
            package="abu_yolo_ros",
            executable="kfs_3d_localizer_node",
            name="kfs_3d_localizer_node",
            output="screen",
            parameters=[config_file]
        ),

        Node(
            package="abu_yolo_ros",
            executable="kfs_localization_stabilizer_node",
            name="kfs_localization_stabilizer_node",
            output="screen",
            parameters=[config_file]
        )
    ])
