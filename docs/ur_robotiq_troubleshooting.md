## UR5e + Robotiq 2F-85 통합 트러블슈팅 & 작업 과정 정리

이 문서는 **UR5e에 Robotiq 2F-85 그리퍼를 부착하고, Gazebo + MoveIt2 + ros2_control 환경에서 정상 동작**시키기까지의  
문제/원인/해결 과정을 정리한 것입니다.

---

### 1. 외부 런치 의존 문제 (robot_description 충돌)

- **문제**
  - 초기에 `ur_simulation_gz/ur_sim_moveit.launch.py` 와 `ur_moveit_config/ur_moveit.launch.py` 를 그대로 사용하면서,
    `ur_picking` 쪽에서 `description_package`, `description_file` 등을 덮어쓰는 방식으로 URDF를 바꾸려 했습니다.
  - 하지만 외부 런치 내부에서도 여러 번 `robot_description` 을 세팅하고 있었고,
    ROS 2/패키지 버전이 조금만 달라져도
    `robot_description` 타입/포맷이 서로 꼬여서 다음과 같은 에러가 반복적으로 발생했습니다.
    - `Unable to parse the value of parameter robot_description as yaml`

- **원인**
  - 어떤 런치는 `Command(...)` substitution으로 `robot_description`을 세팅하는 반면,
  - 다른 런치는 YAML 파일을 그대로 param에 넣으려 하거나, 타입 힌트를 주지 않는 등 **설정 포맷이 섞여 있었음**.
  - 이 상태에서 `LaunchConfiguration`/Override를 통해 description 패키지를 바꾸면,
    어느 한 지점에서라도 `robot_description`이 잘못 세팅되면 전체가 깨지는 구조였습니다.

- **해결**
  - 외부 런치에 의존하지 않고, **`ur_picking` 내부에 전용 통합 런치**를 만들었습니다.
    - `launch/ur_sim_moveit_robotiq.launch.py`
  - 이 런치 하나에서:
    - URDF(Xacro) → `Command([...])`
    - `robot_description` → `{"robot_description": ParameterValue(Command(...), value_type=str)}`
    - `robot_description_semantic` → 동일하게 `ParameterValue(..., str)`
    로 **한 번만, 일관되게** 세팅하도록 변경했습니다.

---

### 2. URDF/파라미터 외부 의존 문제 (ur_description 버전 의존성)

- **문제**
  - 초기에 URDF는 `/opt/ros/humble/share/ur_description/urdf` 에 있는 `ur.urdf.xacro` 구조를 그대로 사용했고,
    파라미터(YAML)는 역시 `ur_description/config/...` 를 참조했습니다.
  - 이 상태로 그리퍼를 붙이면, 나중에 `ur_description` 패키지가 업데이트/변경될 때  
    로컬 프로젝트가 같이 깨질 가능성이 높았습니다.

- **해결**
  - URDF 상단에서 사용하는 파라미터 경로를 모두 **`ur_picking/config` 기준으로 변경**했습니다.
    - `joint_limit_params` → `$(find ur_picking)/config/ur5e/joint_limits.yaml`
    - `kinematics_params` → `$(find ur_picking)/config/ur5e/default_kinematics.yaml`
    - `physical_params` → `$(find ur_picking)/config/ur5e/physical_parameters.yaml`
    - `visual_params` → `$(find ur_picking)/config/ur5e/visual_parameters.yaml`
    - `initial_positions_file` → `$(find ur_picking)/config/initial_positions.yaml`
  - 이렇게 하면, **UR 메시/공통 매크로는 여전히 `ur_description` 것**을 쓰되,
    로봇별 파라미터는 전부 프로젝트 로컬에서 관리하게 되어 버전 의존성이 줄어듭니다.

---

### 3. Gazebo에서 Robotiq 메쉬를 찾지 못하는 문제

- **문제**
  - Ignition Gazebo 로그에 다음과 같은 에러가 반복적으로 출력:
    - `Unable to find file with URI [model://robotiq_description/meshes/visual/2f_85/...`
    - `Could not resolve file [model://robotiq_description/meshes/collision/2f_85/...`
  - 그리퍼 모델이 빨간 wireframe 이거나 아예 보이지 않는 현상 발생.

- **원인**
  - Robotiq URDF는 `package://robotiq_description/...` 를 사용하지만,
    Gazebo는 내부적으로 이를 `model://robotiq_description/...` 형태로 해석합니다.
  - 이때 Gazebo는 **리소스 검색 경로(IGN_GAZEBO_RESOURCE_PATH / GZ_SIM_RESOURCE_PATH)** 안에
    `.../share` 디렉터리가 포함되어 있어야 `model://robotiq_description/...` 를  
    `.../share/robotiq_description/...` 로 올바르게 매핑할 수 있습니다.
  - 초기에는 이 경로를 세팅하지 않아서, Gazebo가 Robotiq 패키지를 찾지 못했습니다.

- **해결**
  - `ur_sim_moveit_robotiq.launch.py` 안에서 다음 두 환경 변수를 설정:

    ```python
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
    ```

  - 이렇게 하면 Ignition/Gazebo가 `model://robotiq_description/...` URI를  
    실제 설치 경로인 `.../share/robotiq_description/...` 로 올바르게 해석할 수 있습니다.

---

### 4. 그리퍼 self-collision 으로 인한 MoveIt invalid start state

- **문제**
  - RViz에서 그리퍼 링크들이 빨간색으로 표시되고,
  - MoveIt 로그에:
    - `Skipping invalid start state (invalid state)`
    - `Motion planning start tree could not be initialized!`
  - 이 뜨면서 플래닝이 실패하거나, 플래닝은 되더라도 실행이 거부되는 문제가 있었습니다.

- **원인**
  - Robotiq 2F-85는 양쪽 핑거/이너 너클/팁이 서로 맞물리는 구조라  
    약간만 닫혀 있어도 링크 간 거리가 매우 가깝습니다.
  - 기본 SRDF(UR 전용)만 사용하면, 이들 사이의 self-collision 도 전부 검사 대상이 되어  
    **시작 자세 자체가 self-collision 상태로 판정**될 수 있습니다.

- **해결**
  1. **그리퍼 초기 각도 완화**
     - `2f_85.ros2_control.xacro` 에 `gripper_initial_position` 파라미터를 추가하고,
       기본값을 `0.0`(완전히 열린 상태)로 설정했습니다.
     - ros2_control 초기 state_interface에서 `gripper_closed_position` 대신  
       `gripper_initial_position`을 사용하도록 변경해,  
       시뮬 시작 시 그리퍼가 열려 있는 상태에서 시작하도록 했습니다.

  2. **SRDF에서 그리퍼 self-collision relax**
     - `ur_picking/srdf/ur_robotiq.srdf.xacro` 를 만들고,
       기존 UR MoveIt SRDF 매크로(`ur_moveit_config/srdf/ur_macro.srdf.xacro`)를 include:

       ```xml
       <xacro:include filename="$(find ur_moveit_config)/srdf/ur_macro.srdf.xacro"/>
       <xacro:ur_srdf name="$(arg name)" prefix="$(arg prefix)"/>
       ```

     - 그 아래에, 기존에 사용하던 SRDF 내용을 기반으로  
       UR + Robotiq 전체에 대한 `disable_collisions` 를 추가했습니다.
       - `tool0` ↔ `robotiq_85_*` 링크 사이
       - `robotiq_85_base_link` ↔ 각 knuckle/finger/inner_knuckle/tip
       - 좌/우 핑거 체인끼리
       - `ur_to_robotiq_link`(어댑터)와 wrist/그리퍼 링크 사이
     - 런치에서는 MoveIt SRDF를  
       `ur_moveit_config` 대신 `ur_picking/srdf/ur_robotiq.srdf.xacro` 를 사용하도록 변경했습니다.

이 조합으로, **기본 시작 자세에서 self-collision 으로 invalid start state가 나는 문제를 크게 줄였고**,  
필요 시 RViz에서 `Use Current State`를 Start State로 사용하면 안정적으로 플래닝이 가능해졌습니다.

---

### 5. 컨트롤러/MoveIt 실행 순서 문제

- **문제**
  - MoveIt은 플래닝에 성공했지만, 실행 단계에서:
    - `Action client not connected to action server: joint_trajectory_controller/follow_joint_trajectory`
    - `Failed to send trajectory part 1 of 1 to controller joint_trajectory_controller`
  - 와 같은 에러로 **실행이 ABORTED** 되었습니다.

- **원인**
  - Ignition Gazebo 내부의 ros2_control plugin이 `/controller_manager` 를 띄우고,
    그 다음 joint 컨트롤러들이 `Configured + active` 상태가 되어야 하는데,
  - MoveIt이 그 이전에 실행을 시도하면,
    해당 액션 서버(`joint_trajectory_controller/follow_joint_trajectory`)가 아직 없어서 실패합니다.

- **해결**
  - `ur_sim_moveit_robotiq.launch.py` 에서:
    - Gazebo + ros2_control + 컨트롤러 스포너를 먼저 띄우고,
    - `TimerAction(period=5.0, ...)` 으로 **5초 뒤에 MoveIt(move_group) + RViz + Servo** 를 띄우도록 조정했습니다.

    ```python
    moveit_start = TimerAction(
        period=5.0,
        actions=[
            move_group_node,
            rviz_node,
            servo_node,
        ],
    )
    ```

  - 이로써, 컨트롤러들이 `/controller_manager/list_controllers` 를 통해  
    `Configured and activated` 될 시간을 확보한 뒤 MoveIt이 실행되도록 했습니다.

---

### 6. Robotiq Activation 컨트롤러 (실기 전용)와 시뮬레이션 충돌

- **문제**
  - 시뮬에서 다음 명령을 실행했을 때:

    ```bash
    ros2 run controller_manager spawner robotiq_activation_controller -c /controller_manager
    ```

    아래와 같은 에러 발생:

    - `Failed to activate controller : robotiq_activation_controller`
    - Gazebo 로그:
      - `Not acceptable command interfaces combination:`
      - `Not existing: reactivate_gripper/reactivate_gripper_cmd ...`

- **원인**
  - `2f_85.ros2_control.xacro` 에서 activation 컨트롤러가 사용하는 GPIO:

    ```xml
    <xacro:unless value="${sim_ignition or sim_isaac}">
      <gpio name="reactivate_gripper">
        <command_interface name="reactivate_gripper_cmd" />
        <command_interface name="reactivate_gripper_response" />
      </gpio>
    </xacro:unless>
    ```

  - Ignition 시뮬 모드에서는 `sim_ignition=true` 이므로,
    위 블록이 **실행되지 않아 GPIO 인터페이스가 생성되지 않습니다.**
  - `robotiq_activation_controller` 는 이 GPIO를 필수로 요구하기 때문에,  
    시뮬에서 이 컨트롤러를 켜면 항상 실패합니다.

- **해결**
  - 시뮬레이션에서는 **`robotiq_activation_controller` 를 전혀 사용하지 않기로 결정**했습니다.
    - 실제 하드웨어용 활성화 컨트롤러이므로 시뮬에는 불필요.
  - 대신, **그리퍼 동작은 `robotiq_gripper_controller` (GripperActionController)** 만 사용:
    - 컨트롤러 YAML (`ur5e_robotiq_2f85_controllers.yaml`) 에 이미 정의되어 있으며,
    - 런치에서 자동으로 스폰되도록 추가했습니다.

---

### 7. 최종 구조: UR5e + Robotiq 2F-85 부착 성공까지의 전체 과정

1. **기본 환경 준비**
   - `Universal_Robots_ROS2_Driver`, `ros2_robotiq_gripper` 소스 패키지 추가
   - Humble + MoveIt2 + ros_gz_sim 설치

2. **URDF 통합**
   - `ur_picking/urdf/ur5e_robotiq_2f85.urdf.xacro` 작성:
     - `ur_description/urdf/ur_macro.xacro` include
     - 로컬 어댑터 `ur_to_robotiq_adapter.urdf.xacro` include
     - `robotiq_description/urdf/robotiq_2f_85_macro.urdf.xacro` include
     - `tool0` 아래에 어댑터 + 그리퍼 부착
     - UR 파라미터/initial_positions를 `ur_picking/config/...` 기준으로 변경

3. **ros2_control/컨트롤러 설정**
   - `ur5e_robotiq_2f85_controllers.yaml` 작성:
     - UR 팔 컨트롤러(joint_trajectory_controller 등)
     - Robotiq용 GripperActionController(`robotiq_gripper_controller`)
   - Ignition 시뮬 모드에서 activation 컨트롤러는 사용하지 않기로 결정

4. **MoveIt SRDF 통합**
   - `ur_picking/srdf/ur_robotiq.srdf.xacro` 작성:
     - `ur_moveit_config/srdf/ur_macro.srdf.xacro` include
     - `<xacro:ur_srdf ...>` 로 UR 기본 그룹/포즈/충돌 설정 적용
     - 그 아래에 Robotiq 관련 `disable_collisions` 추가 (기존 SRDF 참고)
   - 런치에서 MoveIt SRDF 입력을 `ur_robotiq.srdf.xacro` 로 교체

5. **Gazebo + MoveIt 통합 런치 구현**
   - `ur_sim_moveit_robotiq.launch.py` 작성:
     - Xacro → `Command(...)` → `ParameterValue(..., str)` 로 `robot_description` 생성
     - Ignition Gazebo + ros_gz_bridge
     - ros2_control / 컨트롤러 스포너 (팔 + 그리퍼)
     - MoveIt `move_group` + RViz + Servo
     - 리소스 경로(IGN/GZ *_RESOURCE_PATH)에 `robotiq_description` 상위 share 디렉토리 추가
     - TimerAction으로 MoveIt 기동을 컨트롤러 활성화 이후로 지연

6. **테스트 및 튜닝**
   - Gazebo에서 UR5e + Robotiq 모델이 정상 표시되는지 확인
   - `ros2 control list_controllers` 로 컨트롤러 상태 확인
   - `robotiq_gripper_controller/gripper_cmd` 액션으로 그리퍼 열기/닫기 테스트
   - MoveIt에서 `ur_manipulator` 그룹으로 플래닝/실행 테스트
   - 필요 시 RViz에서 Start State를 `Use Current State` 로 갱신해 self-collision 이슈 완화

이 과정을 통해, 현재는 **하나의 런치(`ur_sim_moveit_robotiq.launch.py`)로  
UR5e + Robotiq 2F-85 + Gazebo + MoveIt + ros2_control을 안정적으로 사용하는 구조**를 갖추게 되었습니다.


