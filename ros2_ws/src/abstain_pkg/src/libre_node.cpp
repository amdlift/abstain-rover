#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Adjust depending on your system
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

class LibreNode : public rclcpp::Node
{
public:
    LibreNode(const std::string &port="/dev/ttyACM0", int baud=115200)
    : Node("libre_node"), serial_port_(port), baud_rate_(baud)
    {
        // Publisher for sensor data
        sensor_pub_ = this->create_publisher<std_msgs::msg::String>("sensor_data", 10);

        // Subscriber for servo commands
        servo_sub_ = this->create_subscription<std_msgs::msg::String>(
            "servo_cmds", 10,
            std::bind(&LibreNode::servo_callback, this, std::placeholders::_1)
        );

        motor_axis_sub_ = this->create_subscription<std_msgs::msg::String>(
            "motor_axis", 10,
            std::bind(&LibreNode::motor_axis_callback, this, std::placeholders::_1)
        );

        motor_claw_sub_ = this->create_subscription<std_msgs::msg::String>(
            "motor_claw", 10,
            std::bind(&LibreNode::motor_claw_callback, this, std::placeholders::_1)
        );

        // Open serial
        open_serial();

        // Start thread to read sensor data from Pico
        reader_thread_ = std::thread(&LibreNode::serial_read_loop, this);
    }

    ~LibreNode() {
        run_reader_ = false;
        if (reader_thread_.joinable()) reader_thread_.join();
        close(fd_);
    }

private:
    void open_serial() {
        fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", serial_port_.c_str());
            return;
        }

        struct termios tty;
        tcgetattr(fd_, &tty);
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
        tty.c_iflag &= ~IGNBRK;                     // disable break processing
        tty.c_lflag = 0;                            // no signaling chars, no echo
        tty.c_oflag = 0;                            // no remapping, no delays
        tty.c_cc[VMIN]  = 0;                        // non-blocking read
        tty.c_cc[VTIME] = 10;                       // 1 sec read timeout

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tcsetattr(fd_, TCSANOW, &tty);
        RCLCPP_INFO(this->get_logger(), "Serial port %s opened", serial_port_.c_str());
    }

    void servo_callback(const std_msgs::msg::String::SharedPtr msg) {
        // Forward servo command to Pico
        std::string cmd = msg->data + "\n";
        if (fd_ >= 0) {
            write(fd_, cmd.c_str(), cmd.size());
        }
        RCLCPP_INFO(this->get_logger(), "Forwarded servo command: %s", cmd.c_str());
    }

    void motor_axis_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string cmd = "AXIS:" + msg->data + "\n";
        if (fd_ >= 0) {
            write(fd_, cmd.c_str(), cmd.size());
        }
        RCLCPP_INFO(this->get_logger(), "Forwarded motor axis command: %s", cmd.c_str());
    }

    void motor_claw_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string cmd = "CLAW:" + msg->data + "\n";
        if (fd_ >= 0) {
            write(fd_, cmd.c_str(), cmd.size());
        }
        RCLCPP_INFO(this->get_logger(), "Forwarded motor claw command: %s", cmd.c_str());
    }


    void serial_read_loop() {
        char buf[256];
        std::string line;

        while (rclcpp::ok() && run_reader_) {
            int n = read(fd_, buf, sizeof(buf));
            if (n > 0) {
                for (int i = 0; i < n; ++i) {
                    char c = buf[i];
                    if (c == '\n') {
                        if (!line.empty()) {
                            publish_sensor_line(line);
                            line.clear();
                        }
                    } else {
                        line += c;
                    }
                }
            }
            std::this_thread::sleep_for(5ms); // small sleep to reduce CPU usage
        }
    }

    void publish_sensor_line(const std::string &line) {
        auto msg = std_msgs::msg::String();
        msg.data = line;
        sensor_pub_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Sensor: %s", line.c_str());
    }

    // ROS2
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sensor_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr servo_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motor_axis_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motor_claw_sub_;


    // Serial
    std::string serial_port_;
    int baud_rate_;
    int fd_;
    std::thread reader_thread_;
    bool run_reader_ = true;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LibreNode>("/dev/ttyACM0");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
