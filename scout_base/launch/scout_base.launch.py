from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # The robot can be controlled either through CAN bus or UART port.
    # Make sure the hardware interface is set up correctly before attempting to connect to the robot.

    # You only need to specify the port name, such as "can0", "/dev/ttyUSB0".
    # The port should operate with the following configuration:
    #  * CAN bus: 500k
    #  * UART: 115200, Parity None, 8-bit Data, 1 Stop Bit

    port_name_arg = DeclareLaunchArgument(
        name="port_name",
        default_value="can0",
    )
    is_scout_mini_arg = DeclareLaunchArgument(
        name="is_scout_mini",
        default_value="false",
    )
    simulated_robot_arg = DeclareLaunchArgument(
        name="simulated_robot",
        default_value="false",
    )
    odom_topic_name_arg = DeclareLaunchArgument(
        name="odom_topic_name",
        default_value="odom",
    )
    pub_tf_arg = DeclareLaunchArgument(
        name="pub_tf",
        default_value="$(arg pub_tf)",
    )

    scout_base_node_node = Node(
        package='scout_base',
        executable='scout_base_node',
        output='screen',
    )

    return LaunchDescription([
        port_name_arg,
        is_scout_mini_arg,
        simulated_robot_arg,
        odom_topic_name_arg,
        pub_tf_arg,
        scout_base_node_node
    ])
