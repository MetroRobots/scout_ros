from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_path = get_package_share_path('scout_description')
    robot_description_content = ParameterValue(Command(['xacro ',
                                                       str(description_path / 'urdf' / 'scout_v2.xacro')]),
                                               value_type=str)

#    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" ></node>
    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                      }])

    return LaunchDescription([robot_state_publisher_node])
