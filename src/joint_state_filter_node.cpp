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

    // UR5e 관절 이름들(접두사 없는 기본 이름) 선언
    base_joint_names_ = {
      "shoulder_pan_joint",
      "shoulder_lift_joint",
      "elbow_joint",
      "wrist_1_joint",
      "wrist_2_joint",
      "wrist_3_joint",
    };
    for (const auto & name : base_joint_names_)
    {
      // 기본값 0.0, launch / YAML 에서 override 가능
      this->declare_parameter<double>(name, 0.0);
    }

    // 현재 노드 네임스페이스 아래의 joint_states 퍼블리시
    // 예: namespace=robot01 이면 /robot01/joint_states 로 나감
    // MoveIt 은 기본적으로 RELIABLE 구독을 사용하므로, 여기서도 RELIABLE 퍼블리셔로 맞춘다.
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states",
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    // initial_positions.* 파라미터가 있으면 이를 기반으로 초기 joint_states 한 번 퍼블리시
    publish_initial_joint_states_from_parameters();

    // /joint_states (전역) 구독
    // gz_ros2_control 의 joint_state_broadcaster 는 기본적으로 RELIABLE 를 사용하므로
    // 여기서도 RELIABLE QoS 로 맞춘다.
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",  // 절대 토픽 이름
      rclcpp::QoS(rclcpp::KeepLast(100)).reliable(),
      std::bind(&JointStateFilterNode::jointStateCallback, this, std::placeholders::_1));
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

  void publish_initial_joint_states_from_parameters()
  {
    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;

    // 미리 알고 있는 UR 관절 이름들에 대해 파라미터 조회
    for (const auto & base_name : base_joint_names_)
    {
      rclcpp::Parameter param;
      if (!this->get_parameter(base_name, param))
      {
        continue;
      }

      // double 타입만 initial position 으로 사용
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        continue;
      }

      double value = param.as_double();

      // joint_prefix 가 있으면 prefix 를 붙여서 최종 조인트 이름 구성
      std::string full_name =
        joint_prefix_.empty() ? base_name : (joint_prefix_ + base_name);

      joint_names.push_back(full_name);
      joint_positions.push_back(value);
    }

    if (joint_names.empty())
    {
      RCLCPP_INFO(get_logger(), "No initial joint positions found in parameters. Skipping initial JointState publish.");
      return;
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name = joint_names;
    msg.position = joint_positions;

    RCLCPP_INFO(get_logger(), "Publishing initial JointState with %zu joints.", joint_names.size());
    joint_state_pub_->publish(msg);
  }

  std::string joint_prefix_;
  std::vector<std::string> base_joint_names_;
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


