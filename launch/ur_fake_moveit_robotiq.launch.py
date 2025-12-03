import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare

from ur_moveit_config.launch_common import load_yaml


def launch_setup(context, *args, **kwargs):
    # 기본 인자
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    prefix = LaunchConfiguration("prefix")
    tf_prefix = LaunchConfiguration("tf_prefix")

    robot_id = LaunchConfiguration("robot_id")

    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")

    description_package = "ur_picking"
    description_file = "ur5e_robotiq_2f85.urdf.xacro"
    runtime_config_package = "ur_picking"

    # ros2_control 컨트롤러 YAML (robot_id 별 디렉토리 사용)
    controllers_file_path = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "config",
            robot_id,
            "ur5e_robotiq_2f85_controllers.yaml",
        ]
    )

    # UR 파라미터 (로컬 ur_picking/config/ur5e 사용)
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
        [
            FindPackageShare(description_package),
            "config",
            robot_id,
            "initial_positions.yaml",
        ]
    )

    # robot_description (UR+그리퍼) 생성 - Gazebo 없음, fake_hardware 사용
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
            "tf_prefix:=",
            tf_prefix,
            " ",
            "use_fake_hardware:=true",
            " ",
            "fake_sensor_commands:=true",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content, value_type=str
        )
    }

    # ros2_control 노드 (fake hardware)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ParameterFile(controllers_file_path, allow_substs=True),
        ],
        output="screen",
    )

    # robot_state_publisher (joint_states → TF)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": use_sim_time}, robot_description],
        remappings=[
            ("joint_states", f"/{robot_id.perform(context)}/joint_states"),
        ],
    )

    # JointState 필터 노드 (전역 /joint_states -> /<robot_id>/joint_states)
    robot_id_value = robot_id.perform(context)
    joint_prefix_value = prefix.perform(context)

    initial_positions_yaml = load_yaml(
        "ur_picking",
        os.path.join("config", robot_id_value, "initial_positions.yaml"),
    )

    joint_state_filter_node = Node(
        package="ur_picking",
        executable="joint_state_filter_node",
        name="joint_state_filter_node",
        namespace=robot_id,
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"joint_prefix": joint_prefix_value},
            initial_positions_yaml,
        ],
    )

    # 컨트롤러 스포너 (fake_hardware에서도 ros2_control 컨트롤러를 명시적으로 로드해야 함)
    joint_state_broadcaster_name = f"{robot_id_value}_joint_state_broadcaster"
    joint_trajectory_controller_name = f"{robot_id_value}_joint_trajectory_controller"
    robotiq_gripper_controller_name = f"{robot_id_value}_robotiq_gripper_controller"

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            joint_state_broadcaster_name,
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            joint_trajectory_controller_name,
            "-c",
            "/controller_manager",
        ],
        output="screen",
    )

    robotiq_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            robotiq_gripper_controller_name,
            "-c",
            "/controller_manager",
        ],
        output="screen",
    )

    # RViz 설정 파일 (패키지의 rviz/robot_model_only.rviz 사용)
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_picking"), "rviz", "robot_model_only.rviz"]
    )

    # RViz2 노드 (MoveIt 없이 로봇 모델만 확인용)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(launch_rviz),
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    nodes_to_start = [
        ros2_control_node,
        robot_state_publisher_node,
        joint_state_filter_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        robotiq_gripper_controller_spawner,
        rviz_node,
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
            "robot_id",
            default_value="robot01",
            description="Robot ID / namespace for multi-robot setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix of the joint names.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="TF frame prefix (used to make frame names unique).",
        )
    )

    # 시간 / RViz 관련
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time for all nodes.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz2 for robot visualization?",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


