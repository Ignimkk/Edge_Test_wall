#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class JointStateFilterNode : public rclcpp::Node
{
public:
  JointStateFilterNode()
  : Node("joint_state_filter_node")
  {
    // 파라미터: 필터할 조인트 prefix (예: "robot01_")
    this->declare_parameter<std::string>("joint_prefix", "");
    joint_prefix_ = this->get_parameter("joint_prefix").as_string();

    if (joint_prefix_.empty())
    {
      RCLCPP_WARN(get_logger(),
                  "Parameter 'joint_prefix' is empty. JointStateFilterNode will pass through all joints.");
    }
    else
    {
      RCLCPP_INFO(get_logger(), "JointStateFilterNode using joint_prefix='%s'",
                  joint_prefix_.c_str());
    }

    // /joint_states (전역) 구독
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",  // 절대 토픽 이름
      rclcpp::SensorDataQoS(),
      std::bind(&JointStateFilterNode::jointStateCallback, this, std::placeholders::_1));

    // 현재 노드 네임스페이스 아래의 joint_states 퍼블리시
    // 예: namespace=robot01 이면 /robot01/joint_states 로 나감
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS());
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!msg)
    {
      return;
    }

    // prefix가 없으면 그대로 패스쓰루
    if (joint_prefix_.empty())
    {
      joint_state_pub_->publish(*msg);
      return;
    }

    // prefix로 필터링
    sensor_msgs::msg::JointState filtered;
    filtered.header = msg->header;

    const auto & names = msg->name;
    const auto & positions = msg->position;
    const auto & velocities = msg->velocity;
    const auto & efforts = msg->effort;

    const bool has_pos = !positions.empty();
    const bool has_vel = !velocities.empty();
    const bool has_eff = !efforts.empty();

    for (size_t i = 0; i < names.size(); ++i)
    {
      const std::string & joint_name = names[i];
      if (!joint_prefix_.empty() &&
          joint_name.rfind(joint_prefix_, 0) != 0)  // prefix로 시작하지 않으면 스킵
      {
        continue;
      }

      filtered.name.push_back(joint_name);

      if (has_pos && i < positions.size())
      {
        filtered.position.push_back(positions[i]);
      }
      if (has_vel && i < velocities.size())
      {
        filtered.velocity.push_back(velocities[i]);
      }
      if (has_eff && i < efforts.size())
      {
        filtered.effort.push_back(efforts[i]);
      }
    }

    if (!filtered.name.empty())
    {
      joint_state_pub_->publish(filtered);
    }
  }

  std::string joint_prefix_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointStateFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


