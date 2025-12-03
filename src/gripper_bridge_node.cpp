#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <control_msgs/action/gripper_command.hpp>

class GripperBridgeNode : public rclcpp::Node
{
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GripperCommandGoalHandle = rclcpp_action::ClientGoalHandle<GripperCommand>;

  GripperBridgeNode()
  : Node("gripper_bridge_node")
  {
    RCLCPP_INFO(this->get_logger(), "Gripper Bridge Node initialized");

    // 파라미터: open/close 위치 및 max_effort
    this->declare_parameter<double>("open_position", 0.0);
    this->declare_parameter<double>("close_position", 0.7);
    this->declare_parameter<double>("max_effort", 40.0);

    open_position_ = this->get_parameter("open_position").as_double();
    close_position_ = this->get_parameter("close_position").as_double();
    max_effort_ = this->get_parameter("max_effort").as_double();

    // 로봇 ID 파라미터(예: "robot01", "robot02")를 사용해
    // 로봇별 그리퍼 컨트롤러 액션 이름을 구성한다.
    this->declare_parameter<std::string>("robot_id", "robot01");
    const std::string robot_id = this->get_parameter("robot_id").as_string();

    const std::string controller_name = robot_id + "_robotiq_gripper_controller";
    const std::string action_name = "/" + controller_name + "/gripper_cmd";

    gripper_action_client_ = rclcpp_action::create_client<GripperCommand>(this, action_name);

    RCLCPP_INFO(this->get_logger(), "Waiting for gripper action server: %s", action_name.c_str());
    gripper_action_client_->wait_for_action_server();
    RCLCPP_INFO(this->get_logger(), "Gripper action server connected.");

    // /gripper_open 토픽 구독 (상대 이름, 네임스페이스는 launch에서 remap)
    gripper_cmd_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "gripper_open", 10,
      std::bind(&GripperBridgeNode::gripper_cmd_callback, this, std::placeholders::_1));
  }

private:
  void gripper_cmd_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    // true: open, false: close
    const bool open = msg->data;
    const double target_position = open ? open_position_ : close_position_;

    RCLCPP_INFO(this->get_logger(),
                "Received gripper_open=%s -> target_position=%.3f",
                open ? "true" : "false", target_position);

    GripperCommand::Goal goal;
    goal.command.position = target_position;
    goal.command.max_effort = max_effort_;

    auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
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

    gripper_action_client_->async_send_goal(goal, send_goal_options);
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_cmd_sub_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;

  double open_position_;
  double close_position_;
  double max_effort_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


