import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from ur_moveit_config.launch_common import load_yaml


def launch_setup(context, *args, **kwargs):
    # UR + 시뮬레이션 인자
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    prefix = LaunchConfiguration("prefix")

    # MoveIt / 기타 인자
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")

    # Gazebo 관련 인자
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")

    # 경로 설정
    description_package = "ur_picking"
    description_file = "ur5e_robotiq_2f85.urdf.xacro"
    runtime_config_package = "ur_picking"
    controllers_file = "ur5e_robotiq_2f85_controllers.yaml"

    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    srdf_package = LaunchConfiguration("srdf_package")
    srdf_file = LaunchConfiguration("srdf_file")

    # ros2_control 컨트롤러 YAML
    controllers_file_path = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    # UR 파라미터 (로컬 ur_picking/config 사용)
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "ur5e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "ur5e", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "ur5e", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "ur5e", "visual_parameters.yaml"]
    )
    initial_positions_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "initial_positions.yaml"]
    )

    # robot_description (UR+그리퍼) 생성
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "initial_positions_file:=",
            initial_positions_file,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur",
            " ",
            "prefix:=",
            prefix,
            " ",
            "sim_ignition:=true",
            " ",
            "simulation_controllers:=",
            controllers_file_path,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content, value_type=str
        )
    }

    # robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    # Gazebo / ros_gz_sim
    # Gazebo의 model://robotiq_description/... URI를 해석하려면
    # 상위 디렉터리(share)가 리소스 경로에 포함되어야 함.
    robotiq_share_parent = PathJoinSubstitution(
        [FindPackageShare("robotiq_description"), ".."]
    )
    set_ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[
            robotiq_share_parent,
            ":",
            EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
        ],
    )
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            robotiq_share_parent,
            ":",
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
        ],
    )

    gz_launch_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": [" -r -v 4 ", world_file],
        }.items(),
        condition=IfCondition(gazebo_gui),
    )

    gz_launch_without_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": [" -s -r -v 4 ", world_file],
        }.items(),
        condition=UnlessCondition(gazebo_gui),
    )

    # Gazebo에 UR+그리퍼 엔티티 스폰
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "ur",
            "-allow_renaming",
            "true",
        ],
    )

    # /clock 브리지
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        output="screen",
    )

    # 컨트롤러 스포너
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    start_joint_controller = LaunchConfiguration("start_joint_controller")

    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(start_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(start_joint_controller),
    )

    # Robotiq 그리퍼 액션 컨트롤러 (시뮬에서는 activation 컨트롤러는 사용하지 않음)
    robotiq_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "-c", "/controller_manager"],
    )

    # MoveIt 설정
    # SRDF
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(srdf_package), "srdf", srdf_file]
            ),
            " ",
            "name:=",
            "ur",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        )
    }

    publish_robot_description_semantic = {
        "publish_robot_description_semantic": LaunchConfiguration(
            "publish_robot_description_semantic"
        )
    }

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("ur_picking"), "config", "kinematics.yaml"]
    )

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            str(moveit_config_package.perform(context)),
            os.path.join("config", str(moveit_joint_limits_file.perform(context))),
        )
    }

    # Planning 설정
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution 설정
    controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
    change_controllers = context.perform_substitution(use_sim_time)
    if change_controllers == "true":
        controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
        controllers_yaml["joint_trajectory_controller"]["default"] = True

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    # MoveIt move_group 노드
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            publish_robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
            warehouse_ros_config,
        ],
    )

    # RViz (MoveIt 뷰)
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_picking"), "rviz", "robot_model.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
            warehouse_ros_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Servo 노드
    servo_yaml = load_yaml("ur_moveit_config", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        condition=IfCondition(launch_servo),
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
        ],
        output="screen",
    )

    # Gazebo + 컨트롤러 기동 후 일정 시간 뒤에 MoveIt + RViz + Servo 시작
    moveit_start = TimerAction(
        period=5.0,
        actions=[
            move_group_node,
            rviz_node,
            servo_node,
        ],
    )

    nodes_to_start = [
        set_ign_resource_path,
        set_gz_resource_path,
        gz_launch_with_gui,
        gz_launch_without_gui,
        gz_sim_bridge,
        robot_state_publisher_node,
        gz_spawn_entity,
        joint_state_broadcaster_spawner,
        initial_joint_controller_spawner_started,
        initial_joint_controller_spawner_stopped,
        robotiq_gripper_controller_spawner,
        moveit_start,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    # UR 관련 인자
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type/series of used UR robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names.",
        )
    )

    # 컨트롤러 관련
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_joint_controller",
            default_value="true",
            description="Whether to start the initial joint controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )

    # Gazebo 관련
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
            description="Start Gazebo with GUI?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value="empty.sdf",
            description="Gazebo world file (absolute path or filename in Gazebo collection).",
        )
    )

    # MoveIt / 시간 관련
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time for all nodes.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_robot_description_semantic",
            default_value="True",
            description="Whether to publish the SRDF on /robot_description_semantic.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "srdf_package",
            default_value="ur_picking",
            description="Package that contains the SRDF/XACRO file used to build robot_description_semantic.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "srdf_file",
            default_value="ur_robotiq.srdf.xacro",
            description="SRDF/XACRO file (relative to srdf/ in srdf_package) that defines MoveIt semantics including gripper.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_joint_limits_file",
            default_value="joint_limits.yaml",
            description="MoveIt joint limits that augment or override URDF limits.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path where the warehouse database should be stored.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz with MoveIt configuration?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_servo",
            default_value="true",
            description="Launch MoveIt Servo node?",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

