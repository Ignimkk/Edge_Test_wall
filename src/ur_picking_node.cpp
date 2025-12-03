#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include "ur_picking/action/move_to_pose.hpp"

using MoveToPose = ur_picking::action::MoveToPose;
using MoveToPoseGoalHandle = rclcpp_action::ServerGoalHandle<MoveToPose>;

class UrPickingNode : public rclcpp::Node
{
public:
  UrPickingNode()
    : Node("ur_picking_node")
    , move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), "ur_manipulator")
    , stop_requested_(false)
    , action_executing_(false)
  {
    RCLCPP_INFO(this->get_logger(), "UR Picking Node initialized");

    // MoveIt2 설정 (속도/가속도 스케일 낮춰서 더 천천히 움직이도록 설정)
    move_group_.setPlanningTime(10.0);
    move_group_.setNumPlanningAttempts(10);
    move_group_.setMaxVelocityScalingFactor(0.3);
    move_group_.setMaxAccelerationScalingFactor(0.3);

    // Planning group 정보 출력
    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_.getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_.getEndEffectorLink().c_str());

    // 파라미터 선언
    // planner_type: "RRT", "RRTstar", "RRTConnect" 중 하나 (기본: "RRTConnect")
    this->declare_parameter<std::string>("planner_type", "RRTConnect");
    // use_cartesian: true이면 카테시안 경로, false이면 OMPL 플래너 사용
    this->declare_parameter<bool>("use_cartesian", false);

    // /stop 토픽 구독
    stop_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "/stop", 10,
      std::bind(&UrPickingNode::stop_callback, this, std::placeholders::_1));

    // MoveToPose action server 생성
    action_server_ = rclcpp_action::create_server<MoveToPose>(
      this,
      "move_to_pose",
      std::bind(&UrPickingNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&UrPickingNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&UrPickingNode::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "UR Picking Node ready. Action server: move_to_pose");
  }

private:
  void stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      // stop 신호는 "현재 실행 중인 궤적을 즉시 멈추는 원샷 트리거"로만 사용
      RCLCPP_WARN(this->get_logger(), "Stop signal received!");
      stop_requested_ = true;

      // 현재 실행 중인 action이 있으면 취소
      if (action_executing_ && current_goal_handle_)
      {
        RCLCPP_INFO(this->get_logger(), "Stopping current trajectory...");
        move_group_.stop();
        // 실제 abort 처리는 execute_goal() 내부에서 stop_requested_를 보고 수행
      }
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveToPose::Goal> goal)
  {
    (void)uuid;

    // 이미 실행 중이면 거부
    if (action_executing_)
    {
      RCLCPP_WARN(this->get_logger(), "Goal rejected: Action is already executing");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Goal 유효성 검사 (target_pose가 들어와야 함)
    (void)goal;
    RCLCPP_INFO(this->get_logger(), "Goal accepted. Will plan and execute to target pose");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<MoveToPoseGoalHandle> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Goal cancellation requested");
    
    // MoveIt2 stop 호출
    move_group_.stop();
    action_executing_ = false;
    
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<MoveToPoseGoalHandle> goal_handle)
  {
    // 별도 스레드에서 실행
    std::thread{std::bind(&UrPickingNode::execute_goal, this, goal_handle)}.detach();
  }

  void execute_goal(const std::shared_ptr<MoveToPoseGoalHandle> goal_handle)
  {
    action_executing_ = true;
    current_goal_handle_ = goal_handle;

    RCLCPP_INFO(this->get_logger(), "Executing trajectory...");

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveToPose::Result>();
    auto feedback = std::make_shared<MoveToPose::Feedback>();

    // Stop 체크
    if (stop_requested_)
    {
      RCLCPP_WARN(this->get_logger(), "Execution aborted before planning: Stop signal received");
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
      goal_handle->abort(result);
      action_executing_ = false;
      current_goal_handle_.reset();
      stop_requested_ = false;  // 원샷 처리
      return;
    }

    // 1) Planning
    feedback->state = "PLANNING";
    goal_handle->publish_feedback(feedback);

    // use_cartesian 파라미터 확인
    bool use_cartesian = false;
    try
    {
      use_cartesian = this->get_parameter("use_cartesian").as_bool();
    }
    catch (const rclcpp::ParameterTypeException &)
    {
      RCLCPP_WARN(this->get_logger(), "Parameter 'use_cartesian' has invalid type. Using false.");
      use_cartesian = false;
    }

    moveit_msgs::msg::RobotTrajectory trajectory;

    if (use_cartesian)
    {
      // 카테시안 경로 생성
      RCLCPP_INFO(this->get_logger(), "Planning Cartesian path to target pose...");

      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(goal->target_pose.pose);

      const double eef_step = 0.01;      // 엔드이펙터 이동 분해 수준 (m)
      const double jump_threshold = 0.0; // 점프 감지 비활성화

      double fraction = move_group_.computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory);

      if (fraction < 0.99)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Cartesian path planning failed. Achieved fraction: %.3f", fraction);
        result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
        goal_handle->abort(result);
        action_executing_ = false;
        current_goal_handle_.reset();
        return;
      }

      RCLCPP_INFO(this->get_logger(),
                  "Cartesian path planning succeeded. Fraction: %.3f, points: %zu",
                  fraction, trajectory.joint_trajectory.points.size());
    }
    else
    {
      // OMPL 플래너 사용 (RRT / RRT* / RRTConnect 등)
      std::string planner_type = "RRTConnect";
      try
      {
        planner_type = this->get_parameter("planner_type").as_string();
      }
      catch (const rclcpp::ParameterTypeException &)
      {
        RCLCPP_WARN(this->get_logger(),
                    "Parameter 'planner_type' has invalid type. Using 'RRTConnect'.");
        planner_type = "RRTConnect";
      }

      std::string planner_id;
      if (planner_type == "RRT" || planner_type == "rrt")
      {
        planner_id = "RRTkConfigDefault";
      }
      else if (planner_type == "RRTstar" || planner_type == "RRT*" ||
               planner_type == "rrtstar" || planner_type == "rrt*")
      {
        planner_id = "RRTstarkConfigDefault";
      }
      else
      {
        // 기본값: RRTConnect
        planner_id = "RRTConnectkConfigDefault";
      }

      RCLCPP_INFO(this->get_logger(), "Using planner_id: %s", planner_id.c_str());
      move_group_.setPlannerId(planner_id);

      move_group_.setPoseTarget(goal->target_pose);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto plan_result = move_group_.plan(plan);

      if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(), "Planning failed with error code: %d", plan_result.val);
        result->error_code.val = plan_result.val;
        goal_handle->abort(result);
        action_executing_ = false;
        current_goal_handle_.reset();
        return;
      }

      trajectory = plan.trajectory_;
      RCLCPP_INFO(this->get_logger(), "Planning successful. Trajectory points: %zu",
                  trajectory.joint_trajectory.points.size());
    }

    // 2) Trajectory 실행
    feedback->state = "EXECUTING";
    goal_handle->publish_feedback(feedback);

    moveit::core::MoveItErrorCode error_code = move_group_.execute(trajectory);

    // Stop 체크 (실행 중 stop이 요청되었는지)
    if (stop_requested_)
    {
      RCLCPP_WARN(this->get_logger(), "Execution stopped: Stop signal received during execution");
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
      goal_handle->abort(result);
      action_executing_ = false;
      current_goal_handle_.reset();
      stop_requested_ = false;  // 원샷 처리
      return;
    }

    // 결과 설정
    if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Trajectory executed successfully");
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      goal_handle->succeed(result);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Trajectory execution failed with error code: %d", error_code.val);
      result->error_code.val = error_code.val;
      goal_handle->abort(result);
    }

    action_executing_ = false;
    current_goal_handle_.reset();
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_subscription_;
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  
  bool stop_requested_;
  bool action_executing_;
  std::shared_ptr<MoveToPoseGoalHandle> current_goal_handle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UrPickingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
