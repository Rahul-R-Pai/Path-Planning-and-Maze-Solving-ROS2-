import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from scripts import GazeboRosPaths
 
def generate_launch_description():
    package_share_dir = get_package_share_directory("navigation")
    urdf_file = os.path.join(package_share_dir, "urdf", "rover.urdf")
 
    print("\n\nURDF file location: ", package_share_dir, '\n\n')
    print("\n\nURDF file: ", urdf_file, '\n\n')
 
    model_path, plugin_path, media_path = GazeboRosPaths.get_paths()
    print(model_path, plugin_path, media_path)
 
    env = {
        "GAZEBO_MODEL_PATH": model_path,\
        "GAZEBO_PLUGIN_PATH": plugin_path,\
        "GAZEBO_RESOURCE_PATH": media_path,
    }
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["gazebo","-s","libgazebo_ros_factory.so",],
                output="screen",
                additional_env=env,
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-entity","rover","-b","-file", urdf_file, "-x", "0.0", "-y", "0.0", "-z", "0.0",
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                arguments=[urdf_file],
            ),
        ]
    )