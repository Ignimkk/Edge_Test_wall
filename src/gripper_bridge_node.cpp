#include <memory>
#include <string>
#
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#
#include <std_msgs/msg/bool.hpp>
#include <control_msgs/action/gripper_command.hpp>
#
class GripperBridgeNode : public rclcpp::Node
{
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GripperCommandGoalHandle = rclcpp_action::ClientGoalHandle<GripperCommand>;
#
  GripperBridgeNode()
  : Node("gripper_bridge_node")
  {
    RCLCPP_INFO(this->get_logger(), "Gripper Bridge Node initialized");
#
    // 파라미터: open/close 위치 및 max_effort
    this->declare_parameter<double>("open_position", 0.0);
    this->declare_parameter<double>("close_position", 0.7);
    this->declare_parameter<double>("max_effort", 40.0);
#
    open_position_ = this->get_parameter("open_position").as_double();
    close_position_ = this->get_parameter("close_position").as_double();
    max_effort_ = this->get_parameter("max_effort").as_double();
#
    // 네임스페이스 없이 고정 액션 이름 사용
    const std::string action_name = "/robotiq_gripper_controller/gripper_cmd";
#
    gripper_action_client_ =
      rclcpp_action::create_client<GripperCommand>(this, action_name);
#
    RCLCPP_INFO(
      this->get_logger(),
      "Waiting for gripper action server: %s",
      action_name.c_str());
    gripper_action_client_->wait_for_action_server();
    RCLCPP_INFO(this->get_logger(), "Gripper action server connected.");
#
    // /gripper_open 토픽 구독 (네임스페이스 사용 안 함)
    gripper_cmd_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/gripper_open", 10,
      std::bind(&GripperBridgeNode::gripper_cmd_callback, this, std::placeholders::_1));
  }
#
private:
  void gripper_cmd_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool open = msg->data;  // true: open, false: close
    const double target_position = open ? open_position_ : close_position_;
#
    RCLCPP_INFO(
      this->get_logger(),
      "Received gripper_open=%s -> target_position=%.3f",
      open ? "true" : "false", target_position);
#
    GripperCommand::Goal goal;
    goal.command.position = target_position;
    goal.command.max_effort = max_effort_;
#
    auto send_goal_options =
      rclcpp_action::Client<GripperCommand>::SendGoalOptions();
#
    send_goal_options.goal_response_callback =
      [this](GripperCommandGoalHandle::SharedPtr handle)
      {
        if (!handle)
        {
          RCLCPP_ERROR(this->get_logger(), "Gripper goal was rejected by server");
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), "Gripper goal accepted");
        }
      };
#
    send_goal_options.result_callback =
      [this](const GripperCommandGoalHandle::WrappedResult & result)
      {
        switch (result.code)
        {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Gripper action succeeded");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Gripper action was aborted");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Gripper action was canceled");
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "Gripper action returned unknown result code");
            break;
        }
      };
#
    gripper_action_client_->async_send_goal(goal, send_goal_options);
  }
#
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_cmd_sub_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;
  double open_position_;
  double close_position_;
  double max_effort_;
};
#
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


