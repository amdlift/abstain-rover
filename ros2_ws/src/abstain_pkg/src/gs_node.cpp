#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class MultiServoPublisher : public rclcpp::Node
{
public:
  MultiServoPublisher()
  : Node("multi_servo_publisher"),
    base_angle_(0.0f),
    shoulder_angle_(45.0f),
    elbow_angle_(90.0f)
  {
    // Create three publishers, one per servo
    base_pub_ = this->create_publisher<std_msgs::msg::Float32>("servo_base", 10);
    shoulder_pub_ = this->create_publisher<std_msgs::msg::Float32>("servo_shoulder", 10);
    elbow_pub_ = this->create_publisher<std_msgs::msg::Float32>("servo_elbow", 10);

    // Each servo can have its own timer frequency for asynchronicity
    base_timer_ = this->create_wall_timer(
      100ms, std::bind(&MultiServoPublisher::base_timer_callback, this));

    shoulder_timer_ = this->create_wall_timer(
      150ms, std::bind(&MultiServoPublisher::shoulder_timer_callback, this));

    elbow_timer_ = this->create_wall_timer(
      200ms, std::bind(&MultiServoPublisher::elbow_timer_callback, this));
  }

private:
  void base_timer_callback()
  {
    auto msg = std_msgs::msg::Float32();
    msg.data = base_angle_;
    base_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Base servo: %.2f", msg.data);
    base_angle_ += 1.0f;
    if (base_angle_ > 180.0f) base_angle_ = 0.0f;
  }

  void shoulder_timer_callback()
  {
    auto msg = std_msgs::msg::Float32();
    msg.data = shoulder_angle_;
    shoulder_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Shoulder servo: %.2f", msg.data);
    shoulder_angle_ += 0.5f;
    if (shoulder_angle_ > 180.0f) shoulder_angle_ = 45.0f;
  }

  void elbow_timer_callback()
  {
    auto msg = std_msgs::msg::Float32();
    msg.data = elbow_angle_;
    elbow_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Elbow servo: %.2f", msg.data);
    elbow_angle_ += 0.8f;
    if (elbow_angle_ > 180.0f) elbow_angle_ = 90.0f;
  }

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr base_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr shoulder_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr elbow_pub_;

  // Timers
  rclcpp::TimerBase::SharedPtr base_timer_;
  rclcpp::TimerBase::SharedPtr shoulder_timer_;
  rclcpp::TimerBase::SharedPtr elbow_timer_;

  // Servo angles
  float base_angle_;
  float shoulder_angle_;
  float elbow_angle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiServoPublisher>());
  rclcpp::shutdown();
  return 0;
}
