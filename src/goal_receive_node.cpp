#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <action_msgs/msg/goal_status.hpp>

#include "ur_picking/action/move_to_pose.hpp"

using MoveToPose = ur_picking::action::MoveToPose;
using MoveToPoseGoalHandle = rclcpp_action::ClientGoalHandle<MoveToPose>;

class GoalReceiveNode : public rclcpp::Node
{
public:
  GoalReceiveNode()
    : Node("goal_receive_node")
  {
    RCLCPP_INFO(this->get_logger(), "Goal Receive Node initialized");

    // /move_goal 토픽 구독
    goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/move_goal", 10,
      std::bind(&GoalReceiveNode::goal_callback, this, std::placeholders::_1));

    // /current_state 토픽 발행 (action feedback을 외부로 전달)
    current_state_publisher_ = this->create_publisher<std_msgs::msg::String>(
      "/current_state", 10);

    // MoveToPose action client 생성
    action_client_ = rclcpp_action::create_client<MoveToPose>(
      this, "move_to_pose");

    RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    action_client_->wait_for_action_server();

    RCLCPP_INFO(this->get_logger(), "Goal Receive Node ready. Waiting for /move_goal messages...");
  }

private:
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received move_goal: x=%.3f, y=%.3f, z=%.3f",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    // Action goal 생성 (target_pose에 move_goal 좌표 복사)
    auto goal_msg = MoveToPose::Goal();
    goal_msg.target_pose = *msg;

    // Action 전송
    auto send_goal_options = rclcpp_action::Client<MoveToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&GoalReceiveNode::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&GoalReceiveNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&GoalReceiveNode::result_callback, this, std::placeholders::_1);

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(const MoveToPoseGoalHandle::SharedPtr & goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      
      // 상태 발행
      std_msgs::msg::String state_msg;
      state_msg.data = "REJECTED";
      current_state_publisher_->publish(state_msg);
      return;
    }

    goal_handle_ = goal_handle;
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, executing...");
    
    // 상태 발행
    std_msgs::msg::String state_msg;
    state_msg.data = "ACCEPTED";
    current_state_publisher_->publish(state_msg);
  }

  void feedback_callback(
    MoveToPoseGoalHandle::SharedPtr /*goal_handle*/,
    const std::shared_ptr<const MoveToPose::Feedback> feedback)
  {
    // 서버에서 내려주는 state 문자열을 그대로 /current_state 로 전달
    std_msgs::msg::String state_msg;
    state_msg.data = feedback->state;
    current_state_publisher_->publish(state_msg);
  }

  void result_callback(const MoveToPoseGoalHandle::WrappedResult & result)
  {
    std_msgs::msg::String state_msg;
    
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Action succeeded");
        state_msg.data = "SUCCEEDED";
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(this->get_logger(), "Action was aborted");
        state_msg.data = "ABORTED";
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Action was canceled");
        state_msg.data = "CANCELED";
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        state_msg.data = "UNKNOWN";
        break;
    }

    if (result.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Trajectory execution completed successfully");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Trajectory execution failed with error code: %d",
                   result.result->error_code.val);
      state_msg.data = "FAILED";
    }

    current_state_publisher_->publish(state_msg);
    goal_handle_.reset();
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_state_publisher_;
  rclcpp_action::Client<MoveToPose>::SharedPtr action_client_;
  MoveToPoseGoalHandle::SharedPtr goal_handle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalReceiveNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
