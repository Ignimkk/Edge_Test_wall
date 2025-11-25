# UR Picking Package (UR5e + Robotiq 2F-85)

UR5e 로봇과 Robotiq 2F-85 그리퍼를 사용한 picking 모듈입니다.  
ROS 2 Humble, MoveIt2, Ignition Gazebo(ros_gz_sim)를 사용하며,

- **action 기반 제어**
- **OMPL 플래너 선택 (RRT / RRT\* / RRTConnect)**
- **Cartesian 경로 (computeCartesianPath)**

를 지원합니다.

이 패키지는 **외부 UR 시뮬/MoveIt 런치에 의존하지 않고**,  
`ur_picking` 내부의 URDF/SRDF/ros2_control/컨트롤러/Gazebo/MoveIt 런치로
완전히 독립적인 UR5e + Robotiq 2F-85 시뮬레이션 환경을 제공합니다.

---

## 패키지 구조

```bash
ur_picking/
├── CMakeLists.txt
├── package.xml
├── README.md
├── action/
│   └── MoveToPose.action                # Pose 기반 커스텀 Action
├── config/
│   ├── picking_params.yaml              # (옵션) Picking 파라미터
│   ├── initial_positions.yaml           # UR 초기 관절 각도
│   ├── ur5e/
│   │   ├── default_kinematics.yaml
│   │   ├── joint_limits.yaml
│   │   ├── physical_parameters.yaml
│   │   └── visual_parameters.yaml
│   └── ur5e_robotiq_2f85_controllers.yaml  # UR + Robotiq ros2_control 컨트롤러
├── include/
│   └── ur_picking/
├── launch/
│   ├── ur_picking.launch.py             # Picking 노드 런치
│   └── ur_sim_moveit_robotiq.launch.py  # UR+Robotiq + Gazebo + MoveIt 통합 런치
├── srdf/
│   └── ur_robotiq.srdf.xacro            # UR + Robotiq용 MoveIt SRDF (충돌 설정 포함)
├── urdf/
│   ├── ur5e_robotiq_2f85.urdf.xacro     # UR5e + Robotiq 통합 URDF/XACRO
│   └── ur_to_robotiq_adapter.urdf.xacro # UR tool0 ↔ Robotiq 어댑터 링크
└── src/
    ├── ur_picking_node.cpp              # Action Server (MoveIt2 계획 + 궤적 실행)
    └── goal_receive_node.cpp            # Action Client (외부 토픽 ↔ Action 중계)
```

---

## 시스템 아키텍처

### 1. 시뮬레이션 + MoveIt + 그리퍼

`ur_sim_moveit_robotiq.launch.py` 가 다음을 한 번에 띄웁니다.

- Ignition Gazebo + ros_gz_bridge
- UR5e + Robotiq 2F-85 통합 URDF (`ur5e_robotiq_2f85.urdf.xacro`)
- ros2_control + 컨트롤러:
  - `joint_state_broadcaster`
  - `joint_trajectory_controller` (팔 제어)
  - `robotiq_gripper_controller` (GripperActionController, 그리퍼 제어)
- MoveIt2:
  - `move_group` 노드
  - Servo 노드 (옵션)
  - MoveIt RViz (`view_robot.rviz`)

`robot_description` / `robot_description_semantic` 는 모두 런치 내부에서  
Xacro + `ParameterValue(..., value_type=str)` 를 사용해 일관되게 설정합니다.

### 2. Picking 노드

1. **goal_receive_node** (Action Client)
   - `/move_goal` 토픽을 구독하여 외부에서 보낸 목표 Pose 수신
   - 수신한 Pose를 `MoveToPose` Action Goal(`target_pose`)로 변환하여 `ur_picking_node`에 전송
   - Action 서버로부터 받은 feedback/result를 `/current_state`(std_msgs/String)로 퍼블리시

2. **ur_picking_node** (Action Server + MoveIt2)
   - 커스텀 Action `MoveToPose` (`ur_picking/action/MoveToPose.action`) 서버
     - Goal: `geometry_msgs/PoseStamped target_pose`
     - Feedback: `string state` (예: `"PLANNING"`, `"EXECUTING"`)
     - Result: `moveit_msgs/MoveItErrorCodes error_code`
   - MoveIt2 `move_group_interface`를 사용하여:
     - OMPL 플래너(RRT / RRT\* / RRTConnect)
     - 또는 Cartesian 경로(`computeCartesianPath`)
     로 궤적 생성
   - `/stop` 토픽을 구독하여 **현재 실행 중인 궤적을 즉시 정지(원샷 Stop)** 제공

---

## 토픽 및 Action

### 토픽

- **`/move_goal`** (`geometry_msgs/msg/PoseStamped`)
  - 외부에서 목표 Pose(물체 좌표 포함)를 전송하는 토픽
  - `goal_receive_node`가 구독 → `MoveToPose` Action Goal로 변환

- **`/stop`** (`std_msgs/msg/Bool`)
  - 실행 중단 신호
  - `true` 전송 시:
    - 현재 실행 중인 MoveIt 궤적을 `move_group_.stop()` 으로 즉시 정지
    - 해당 Action Goal은 ABORT 처리 (원샷)
  - `false` 를 다시 보낼 필요 없이, 이후 새로운 `/move_goal`을 보내면 바로 다시 이동 가능

- **`/current_state`** (`std_msgs/msg/String`)
  - Action 서버 상태 및 결과를 문자열로 퍼블리시  
  - 예: `"PLANNING"`, `"EXECUTING"`, `"SUCCEEDED"`, `"FAILED"`, `"ABORTED"`, `"REJECTED"` 등

### Action

- **`MoveToPose`** (`ur_picking/action/MoveToPose.action`)
  - Goal:
    - `geometry_msgs/PoseStamped target_pose`
  - Result:
    - `moveit_msgs/MoveItErrorCodes error_code`
  - Feedback:
    - `string state`

---

## 종속성

- ROS 2 Humble
- MoveIt2 (moveit_core, moveit_ros_planning_interface, moveit_ros_move_group 등)
- UR Robot Driver (ur_robot_driver, ur_moveit_config)
- Ignition Gazebo / ros_gz_sim / ros_gz_bridge
- ros2_robotiq_gripper (robotiq_description, robotiq_controllers, robotiq_driver)
- geometry_msgs, std_msgs, moveit_msgs, action_msgs 등

---

## 빌드 방법

1. 워크스페이스 이동

```bash
cd /home/mkketi/dev_ws/Edge/Test_wall_ur
```

2. ROS 2 환경 설정

```bash
source /opt/ros/humble/setup.bash
```

3. 콜콘 빌드

```bash
colcon build --packages-select ur_picking ros2_robotiq_gripper --cmake-args -DCMAKE_BUILD_TYPE=Release
```

4. 워크스페이스 소스

```bash
source install/setup.bash
```

---

## 실행 방법 (UR5e + Robotiq 2F-85 + MoveIt + Picking)

### 1. Gazebo + UR5e + Robotiq + MoveIt 실행

```bash
ros2 launch ur_picking ur_sim_moveit_robotiq.launch.py ur_type:=ur5e safety_limits:=true
```

- Ignition Gazebo GUI가 뜨고, UR5e + Robotiq 2F-85 모델이 스폰됩니다.
- `joint_state_broadcaster`, `joint_trajectory_controller`, `robotiq_gripper_controller`가 자동으로 활성화됩니다.
- MoveIt `move_group` + RViz가 함께 실행됩니다.

### 2. Picking 노드 실행 (플래너/Cartesian 선택)

별도 터미널에서:

```bash
source install/setup.bash

# 조인트-공간 RRT 플래너
ros2 launch ur_picking ur_picking.launch.py use_cartesian:=false planner_type:=RRT

# 조인트-공간 RRT* 플래너 (경로 최적화, 더 느릴 수 있음)
ros2 launch ur_picking ur_picking.launch.py use_cartesian:=false planner_type:=RRTstar

# Cartesian 경로 (엔드이펙터 직선 경로)
ros2 launch ur_picking ur_picking.launch.py use_cartesian:=true
```

### 2-1. 런타임 중 플래너 변경

```bash
# RRT 사용
ros2 param set /ur_picking_node planner_type "RRT"

# RRT* 사용 (두 표기 모두 허용)
ros2 param set /ur_picking_node planner_type "RRTstar"
ros2 param set /ur_picking_node planner_type "RRT*"
```

### 3. move_goal 전송 (기본 예제)

```bash
ros2 topic pub --once /move_goal geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {
    position: {x: 0.0, y: 0.35, z: 0.26},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

엔드이펙터 자세를 바꾸고 싶으면 `orientation`을 적절히 변경하면 됩니다.

### 4. 정지 (Stop)

```bash
ros2 topic pub /stop std_msgs/msg/Bool "data: true"
```

현재 실행 중인 궤적이 즉시 멈추고, 해당 Goal은 ABORT 처리됩니다.  
이후 새로운 `/move_goal`을 바로 보낼 수 있습니다.

### 5. 상태 확인

```bash
ros2 topic echo /current_state
```

`PLANNING`, `EXECUTING`, `SUCCEEDED`, `FAILED`, `ABORTED`, `REJECTED` 등의 상태를 확인할 수 있습니다.

---

## 그리퍼 제어 요약

- 시뮬 실행 시, `robotiq_gripper_controller` 가 자동으로 활성화됩니다.
- GripperActionController 액션으로 직접 제어할 수 있습니다.

```bash
# 현재 컨트롤러 상태 확인
ros2 control list_controllers

# 그리퍼 열기
ros2 action send_goal \
  /robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.0, max_effort: 40.0}}"

# 그리퍼 닫기 (예: 0.7 근처)
ros2 action send_goal \
  /robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.7, max_effort: 40.0}}"
```

시뮬레이션에서는 **`robotiq_activation_controller` 는 사용하지 않습니다.**  
이는 실제 하드웨어 활성화를 위한 GPIO 기반 컨트롤러이며,  
Ignition Gazebo용 ros2_control 구성에서는 해당 GPIO 인터페이스가 생성되지 않기 때문입니다.

---

## 동작 흐름 (최종)

1. 외부 노드에서 `/move_goal` 토픽으로 목표 Pose 전송
2. `goal_receive_node`가 Pose를 수신하고 `MoveToPose` Action Goal(`target_pose`)로 변환
3. `ur_picking_node`가 Goal 수신:
   - `planner_type` / `use_cartesian` 설정에 따라
     - OMPL 플래너(RRT / RRT\* / RRTConnect) 또는
     - Cartesian 경로
     로 계획
4. 계획된 `RobotTrajectory`를 MoveIt2 `move_group`으로 실행
5. 실행 중 `/stop` 토픽으로 `true` 전송 시:
   - 현재 궤적 정지 + 해당 Goal ABORT (원샷)
6. 이후 새로운 `/move_goal` 수신 시 다시 2번부터 반복

---

## 설정

`config/picking_params.yaml` (선택 사용) 파일에서 다음을 설정할 수 있습니다:

- Planning 파라미터 (planning_time, num_planning_attempts 등)
- 홈 포지션 (joint values)
- Pick/Place 포지션 (position, orientation)

---

## 주의사항

- MoveIt2와 UR 로봇 드라이버/시뮬레이터가 **정상적으로 실행 중**이어야 합니다.
- Planning group 이름은 기본적으로 `"ur_manipulator"` 입니다.
- `use_cartesian:=true` 인 경우, 시작 Pose와 목표 Pose 사이에 충돌/IK 실패 등이 있으면 Cartesian 경로 계획이 실패할 수 있습니다.
- Action 기반 제어이므로, `goal_receive_node`와 `ur_picking_node`가 모두 실행되어야 정상 동작합니다.

