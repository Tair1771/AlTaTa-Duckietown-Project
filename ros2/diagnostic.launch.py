from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory("duck2_autonomy")) / "diagnostic.yaml")
    return LaunchDescription([
        Node(package="duck2_autonomy", executable="supervisor.py", parameters=[config]),
        Node(package="duck2_autonomy", executable="autonomy.py", parameters=[config]),
        Node(package="duck2_autonomy", executable="gateway.py"),
    ])
