#include <rclcpp/rclcpp.hpp>

class MotorDriverNode : public rclcpp::Node {
public:
  MotorDriverNode() : Node("motor_driver_node") {
    RCLCPP_INFO(get_logger(), "motor_driver_node starting up");
    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() { RCLCPP_INFO(get_logger(), "tick"); }
    );
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorDriverNode>());
  rclcpp::shutdown();
  return 0;
}
