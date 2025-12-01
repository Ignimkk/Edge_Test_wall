#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class RobotDescriptionPublisherNode : public rclcpp::Node
{
public:
  RobotDescriptionPublisherNode()
  : Node("robot_description_publisher_node")
  {
    // launch 에서 전달되는 robot_description / robot_description_semantic 파라미터
    this->declare_parameter<std::string>("robot_description", "");
    this->declare_parameter<std::string>("robot_description_semantic", "");
    std::string robot_description;
    if (!this->get_parameter("robot_description", robot_description) || robot_description.empty())
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Parameter 'robot_description' is empty or not set. Cannot publish 'robot_description' topic.");
      return;
    }

    std::string robot_description_semantic;
    bool have_semantic =
      this->get_parameter("robot_description_semantic", robot_description_semantic) &&
      !robot_description_semantic.empty();

    // 현재 네임스페이스 정보는 로그용으로만 사용
    const std::string ns = this->get_namespace();
    if (ns.empty())
    {
      RCLCPP_INFO(this->get_logger(),
                  "Publishing robot_description on topic 'robot_description'");
      if (have_semantic)
      {
        RCLCPP_INFO(this->get_logger(),
                    "Publishing robot_description_semantic on topic 'robot_description_semantic'");
      }
    }
    else
    {
      RCLCPP_INFO(this->get_logger(),
                  "Publishing robot_description on topic '%s'",
                  (ns + "/robot_description").c_str());
      if (have_semantic)
      {
        RCLCPP_INFO(this->get_logger(),
                    "Publishing robot_description_semantic on topic '%s'",
                    (ns + "/robot_description_semantic").c_str());
      }
    }

    // latched QoS: 나중에 구독한 노드도 마지막 메시지를 받을 수 있게
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    robot_description_pub_ =
      this->create_publisher<std_msgs::msg::String>("robot_description", qos);
    if (have_semantic)
    {
      robot_description_semantic_pub_ =
        this->create_publisher<std_msgs::msg::String>("robot_description_semantic", qos);
    }

    std_msgs::msg::String msg;
    msg.data = robot_description;
    robot_description_pub_->publish(msg);

    if (have_semantic)
    {
      std_msgs::msg::String smsg;
      smsg.data = robot_description_semantic;
      robot_description_semantic_pub_->publish(smsg);
    }

    RCLCPP_INFO(this->get_logger(), "robot_description published once.");
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_description_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_description_semantic_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotDescriptionPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


