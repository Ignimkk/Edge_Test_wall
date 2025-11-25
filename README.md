<<<<<<< HEAD
# UR Picking Package

UR 로봇을 활용한 picking 모듈입니다. ROS 2 Humble과 MoveIt2를 사용하며, **action 기반 제어 + RRT/RRT\* 플래너 선택 + Cartesian 경로 선택**을 지원합니다.

## 패키지 구조

```
ur_picking/
├── CMakeLists.txt
├── package.xml
├── README.md
├── action/
│   └── MoveToPose.action         # Pose 기반 커스텀 Action
├── config/
│   └── picking_params.yaml       # (옵션) Picking 파라미터 설정
├── include/
│   └── ur_picking/
├── launch/
│   └── ur_picking.launch.py      # Launch 파일
└── src/
    ├── ur_picking_node.cpp       # Action Server (MoveIt2 계획 + 궤적 실행)
    └── goal_receive_node.cpp     # Action Client (외부 토픽 ↔ Action 중계)
```

## 시스템 아키텍처

1. **goal_receive_node** (Action Client)
   - `/move_goal` 토픽을 구독하여 **외부에서 보낸 목표 Pose** 수신
   - 수신한 Pose를 `MoveToPose` Action Goal(`target_pose`)로 변환하여 `ur_picking_node`에 전송
   - Action 서버로부터 받은 **feedback/result를 `/current_state`(std_msgs/String)** 로 퍼블리시

2. **ur_picking_node** (Action Server + MoveIt2)
   - 커스텀 Action `MoveToPose` (`ur_picking/action/MoveToPose.action`) 서버
     - Goal: `geometry_msgs/PoseStamped target_pose`
     - Feedback: `string state` (예: `"PLANNING"`, `"EXECUTING"`)
     - Result: `moveit_msgs/MoveItErrorCodes error_code`
   - MoveIt2 `move_group_interface`를 사용하여:
     - **OMPL 플래너**(RRT / RRT\* / RRTConnect) 또는
     - **Cartesian 경로**(`computeCartesianPath`)
     를 선택적으로 사용해 궤적 생성
   - `/stop` 토픽을 구독하여 **현재 실행 중인 궤적을 즉시 정지하는 원샷 Stop 기능** 제공
     - Stop 후에는 **바로 새로운 `/move_goal`을 보내면 다시 계획/실행 가능**

## 토픽 및 Action

### 토픽

- **`/move_goal`** (`geometry_msgs/msg/PoseStamped`)
  - 외부에서 목표 Pose(물체 좌표 포함)를 전송하는 토픽
  - `goal_receive_node`가 구독 → `MoveToPose` Action Goal로 변환

- **`/stop`** (`std_msgs/msg/Bool`)
  - 실행 중단 신호
  - `true` 전송 시:
    - 현재 실행 중인 MoveIt 궤적을 `move_group_.stop()` 으로 **즉시 정지**
    - 해당 Action Goal은 ABORT 처리 (원샷)
  - **`false` 를 다시 보낼 필요 없이**, 이후 새로운 `/move_goal`을 보내면 바로 다시 이동 가능

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

## 종속성

- ROS 2 Humble
- MoveIt2 (moveit_core, moveit_ros_planning_interface, moveit_ros_move_group 등)
- UR Robot Driver (ur_robot_driver, ur_moveit_config)
- geometry_msgs, std_msgs, moveit_msgs, action_msgs 등

## 빌드 방법

1. 워크스페이스로 이동:

```bash
cd /home/mkketi/dev_ws/Edge/Test_wall_ur
```

2. ROS 2 환경 설정:

```bash
source /opt/ros/humble/setup.bash
```

3. 콜콘 빌드:

```bash
colcon build --packages-select ur_picking --cmake-args -DCMAKE_BUILD_TYPE=Release
```

4. 워크스페이스 소스:

```bash
source install/setup.bash
```

## 실행 방법 (TEST wall 시나리오)

### 1. Gazebo + UR + MoveIt 실행

다른 패키지에서 제공하는 Gazebo + MoveIt 통합 런치 사용:

```bash
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py gazebo_gui:=false
```

### 2. Picking 노드 실행 (플래너/Cartesian 선택)

아래 중 하나를 선택:

```bash
# 조인트-공간 RRT 플래너
ros2 launch ur_picking ur_picking.launch.py use_cartesian:=false planner_type:=RRT

# 조인트-공간 RRT* 플래너 (경로 최적화, 더 느릴 수 있음)
ros2 launch ur_picking ur_picking.launch.py use_cartesian:=false planner_type:=RRTstar

# Cartesian 경로 (엔드이펙터 직선 경로)
ros2 launch ur_picking ur_picking.launch.py use_cartesian:=true
```

### 2-1. 런타임 중 플래너 변경

노드 실행 중에도 `planner_type` 파라미터로 RRT / RRT\* 변경 가능:

```bash
# RRT 사용
ros2 param set /ur_picking_node planner_type "RRT"

# RRT* 사용 (두 표기 모두 허용)
ros2 param set /ur_picking_node planner_type "RRTstar"
ros2 param set /ur_picking_node planner_type "RRT*"
```

### 3. move_goal 전송 (위치를 기준으로 이동)

기본 자세 (엔드이펙터가 "기본 방향"을 보는 상태):

```bash
ros2 topic pub --once /move_goal geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {
    position: {x: 0.0, y: 0.35, z: 0.26},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

### 3-1. 엔드이펙터 자세 변경 (예: 아래를 바라보게)

orientation을 변경해서 자세를 바꿀 수 있습니다.  
예: x축 기준 180도 회전 (툴을 뒤집어서 아래를 보게):

```bash
ros2 topic pub --once /move_goal geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {
    position: {x: 0.0, y: 0.35, z: 0.26},
    orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}
  }
}"
```

### 4. 정지 (Stop)

현재 실행 중인 궤적을 즉시 멈추고, 해당 Action Goal을 ABORT:

```bash
ros2 topic pub /stop std_msgs/msg/Bool "data: true"
```

이후에는 **추가 `/stop false` 없이**, 새로운 `/move_goal`을 보내면 다시 계획/실행합니다.

### 5. 상태 확인

```bash
ros2 topic echo /current_state
```

`PLANNING`, `EXECUTING`, `SUCCEEDED`, `FAILED`, `ABORTED`, `REJECTED` 등의 상태 문자열을 확인할 수 있습니다.

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

## 설정

`config/picking_params.yaml` (선택 사용) 파일에서 다음을 설정할 수 있습니다:
- Planning 파라미터 (planning_time, num_planning_attempts 등)
- 홈 포지션 (joint values)
- Pick/Place 포지션 (position, orientation)

## 주의사항

- MoveIt2와 UR 로봇 드라이버/시뮬레이터(`ur_simulation_gz`)가 **정상적으로 실행 중**이어야 합니다.
- Planning group 이름은 `"ur_manipulator"`로 설정되어 있습니다. 실제 로봇 설정에 맞게 수정이 필요할 수 있습니다.
- `use_cartesian:=true` 인 경우, 시작 Pose와 목표 Pose 사이에 충돌/IK 실패 등이 있으면 Cartesian 경로 계획이 실패할 수 있습니다.
- Action 기반 제어이므로, `goal_receive_node`와 `ur_picking_node`가 모두 실행되어야 정상 동작합니다.
=======
# Edge_Test_wall
>>>>>>> ec26973d012258584956f4681fe7c82b374acb0e
