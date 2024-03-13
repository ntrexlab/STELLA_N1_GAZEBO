import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    
    package_description = 'stella_description'
    robot_model_path = os.path.join(
        get_package_share_directory(package_description))
    xacro_file = os.path.join(robot_model_path, 'urdf', 'stella_slam.urdf.xacro')
    
    # convert XACRO file into URDF
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = doc.toxml()

    params = {'robot_description': robot_description}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[params]
    )

    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'stella_urdf.rviz')

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        name="rviz_node",
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )

    # create and return launch description object
    return LaunchDescription(
        [
            robot_state_publisher_node,
            # rviz_node
        ]
    )