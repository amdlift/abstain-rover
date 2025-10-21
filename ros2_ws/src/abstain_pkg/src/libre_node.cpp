#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <sstream>
#include <mutex>

class PicoServoSubscriber : public rclcpp::Node
{
public:
    PicoServoSubscriber(const std::string & serial_port="/dev/ttyACM0")
    : Node("pico_servo_subscriber")
    {
        // Open serial port
        fd_ = open(serial_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", serial_port.c_str());
        } else {
            configure_port();
            RCLCPP_INFO(this->get_logger(), "Serial port %s opened", serial_port.c_str());
        }

        // Subscribers
        base_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "servo_base", 10,
            [this](std_msgs::msg::Float32::SharedPtr msg){ update_angle("B", msg->data); });

        shoulder_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "servo_shoulder", 10,
            [this](std_msgs::msg::Float32::SharedPtr msg){ update_angle("S", msg->data); });

        elbow_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "servo_elbow", 10,
            [this](std_msgs::msg::Float32::SharedPtr msg){ update_angle("E", msg->data); });
    }

    ~PicoServoSubscriber() {
        if (fd_ >= 0) close(fd_);
    }

private:
    void configure_port() {
        struct termios tty;
        tcgetattr(fd_, &tty);
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
        tty.c_iflag &= ~IGNBRK; // disable break processing
        tty.c_lflag = 0; // no signaling chars, no echo
        tty.c_oflag = 0; // no remapping, no delays
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout
        tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls, enable reading
        tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        tcsetattr(fd_, TCSANOW, &tty);
    }

    void update_angle(const std::string & servo, float value) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (fd_ < 0) return;
        angles_[servo] = value;

        // Format string: "B:90,S:45,E:120\n"
        std::ostringstream oss;
        oss << "B:" << angles_["B"] << ",S:" << angles_["S"] << ",E:" << angles_["E"] << "\n";
        std::string out = oss.str();
        write(fd_, out.c_str(), out.size());
        tcdrain(fd_); // make sure data is sent
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr base_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr shoulder_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr elbow_sub_;

    int fd_;
    std::mutex mutex_;
    std::map<std::string, float> angles_ {{"B",0.0f}, {"S",0.0f}, {"E",0.0f}};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PicoServoSubscriber>("/dev/ttyACM0");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
