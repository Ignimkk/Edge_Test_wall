from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    use_cartesian_arg = DeclareLaunchArgument(
        'use_cartesian',
        default_value='false',
        description='Use Cartesian path (true) or OMPL planner (false)'
    )

    planner_type_arg = DeclareLaunchArgument(
        'planner_type',
        default_value='RRTConnect',
        description='Planner type: RRT, RRTstar, or RRTConnect'
    )

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.100',
        description='Robot IP address'
    )

    # UR Picking Node (Action Server)
    ur_picking_node = Node(
        package='ur_picking',
        executable='ur_picking_node',
        name='ur_picking_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_cartesian': LaunchConfiguration('use_cartesian'),
            'planner_type': LaunchConfiguration('planner_type'),
        }]
    )

    # Goal Receive Node (Action Client)
    goal_receive_node = Node(
        package='ur_picking',
        executable='goal_receive_node',
        name='goal_receive_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # Gripper Bridge Node (토픽 -> 액션 브릿지)
    gripper_bridge_node = Node(
        package='ur_picking',
        executable='gripper_bridge_node',
        name='gripper_bridge_node',
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_cartesian_arg,
        planner_type_arg,
        robot_ip_arg,
        ur_picking_node,
        goal_receive_node,
        gripper_bridge_node,
    ])

