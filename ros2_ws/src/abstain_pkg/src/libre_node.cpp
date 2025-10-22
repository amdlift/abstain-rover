#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <sstream>
#include <mutex>
#include <map>

class PicoServoMotorSubscriber : public rclcpp::Node
{
public:
    PicoServoMotorSubscriber(const std::string & serial_port="/dev/ttyACM0")
    : Node("pico_servo_motor_subscriber")
    {
        // Open serial port
        fd_ = open(serial_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", serial_port.c_str());
        } else {
            configure_port();
            RCLCPP_INFO(this->get_logger(), "Serial port %s opened", serial_port.c_str());
        }

        // ---- Servo Subscribers ----
        wrist_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "servo_wrist", 10,
            [this](std_msgs::msg::Float32::SharedPtr msg){ update_value("W", msg->data); });

        shoulder_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "servo_shoulder", 10,
            [this](std_msgs::msg::Float32::SharedPtr msg){ update_value("S", msg->data); });

        elbow_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "servo_elbow", 10,
            [this](std_msgs::msg::Float32::SharedPtr msg){ update_value("E", msg->data); });

        // ---- Motor Subscribers ----
        axis_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "motor_axis", 10,
            [this](std_msgs::msg::Float32::SharedPtr msg){ update_value("A", msg->data); });

        claw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "motor_claw", 10,
            [this](std_msgs::msg::Float32::SharedPtr msg){ update_value("C", msg->data); });
    }

    ~PicoServoMotorSubscriber() {
        if (fd_ >= 0) close(fd_);
    }

private:
    void configure_port() {
        struct termios tty;
        tcgetattr(fd_, &tty);
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 5;  // 0.5 sec read timeout
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        tcsetattr(fd_, TCSANOW, &tty);
    }

    void update_value(const std::string & id, float value) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (fd_ < 0) return;
        values_[id] = value;

        // Format all current values
        std::ostringstream oss;
        oss << "W:" << values_["W"]
            << ",S:" << values_["S"]
            << ",E:" << values_["E"]
            << ",A:" << values_["A"]
            << ",C:" << values_["C"]
            << "\n";

        std::string out = oss.str();
        write(fd_, out.c_str(), out.size());
        tcdrain(fd_); // ensure all data sent

        RCLCPP_INFO(this->get_logger(), "Sent: %s", out.c_str());
    }

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr wrist_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr shoulder_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr elbow_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr axis_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr claw_sub_;

    // Serial
    int fd_;
    std::mutex mutex_;

    // Latest values for each channel
    std::map<std::string, float> values_{
        {"W", 0.0f}, {"S", 0.0f}, {"E", 0.0f}, {"A", 0.0f}, {"C", 0.0f}
    };
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PicoServoMotorSubscriber>("/dev/ttyACM0");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
