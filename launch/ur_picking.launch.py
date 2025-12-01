from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Launch arguments
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='robot01',
        description='Robot ID / namespace for external interfaces (topics)'
    )

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

    planning_group_arg = DeclareLaunchArgument(
        'planning_group',
        default_value='ur_manipulator',
        description='MoveIt planning group name (e.g., ur_manipulator, robot01_ur_manipulator)'
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
        namespace=LaunchConfiguration('robot_id'),
        output='screen',
        remappings=[
            # /<robot_id>/stop 으로 외부 인터페이스를 분리
            ('stop', ['/', LaunchConfiguration('robot_id'), '/stop']),
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_cartesian': LaunchConfiguration('use_cartesian'),
            'planner_type': LaunchConfiguration('planner_type'),
            'robot_id': LaunchConfiguration('robot_id'),
            'planning_group': LaunchConfiguration('planning_group'),
        }]
    )

    # Goal Receive Node (Action Client)
    goal_receive_node = Node(
        package='ur_picking',
        executable='goal_receive_node',
        name='goal_receive_node',
        namespace=LaunchConfiguration('robot_id'),
        output='screen',
        remappings=[
            # /<robot_id>/move_goal, /<robot_id>/current_state 로 외부 인터페이스를 분리
            ('move_goal', ['/', LaunchConfiguration('robot_id'), '/move_goal']),
            ('current_state', ['/', LaunchConfiguration('robot_id'), '/current_state']),
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_id': LaunchConfiguration('robot_id'),
        }]
    )

    # Gripper Bridge Node (Topic -> GripperCommand Action)
    gripper_bridge_node = Node(
        package='ur_picking',
        executable='gripper_bridge_node',
        name='gripper_bridge_node',
        namespace=LaunchConfiguration('robot_id'),
        output='screen',
        remappings=[
            # /<robot_id>/gripper_open 토픽으로 외부 인터페이스를 분리
            ('gripper_open', ['/', LaunchConfiguration('robot_id'), '/gripper_open']),
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription([
        robot_id_arg,
        use_sim_time_arg,
        use_cartesian_arg,
        planner_type_arg,
        planning_group_arg,
        robot_ip_arg,
        ur_picking_node,
        goal_receive_node,
        gripper_bridge_node,
    ])

