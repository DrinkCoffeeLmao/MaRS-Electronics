#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <algorithm>
#include <memory>

int drive_speed = 0;

class JoyControl : public rclcpp::Node {
public:
    JoyControl() : Node("joy_control") {

        // Subscribe to joystick inputs
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&JoyControl::joyCallback, this, std::placeholders::_1));

        // Publisher for drive speed only
        drive_speed_publisher_ = this->create_publisher<std_msgs::msg::Int16>(
            "/rover_cmd", 10);
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        std_msgs::msg::Int16 drive_speed_msg;

        // ====== SPEED CONTROL ONLY ======

        // START + SELECT = STOP
        if (msg->buttons[12] == 1) {
            drive_speed = 3;
            RCLCPP_INFO(this->get_logger(), "Rover: Stop");
        }

        // A button: Speed 1
        else if (msg->buttons[0] == 1) {
            drive_speed = 1;
            RCLCPP_INFO(this->get_logger(),
                        "Drive: Decrease Speed -> %d", drive_speed);
        }

        // LB button: Speed 2
        else if (msg->buttons[4] == 1) {
            drive_speed = 2;
            RCLCPP_INFO(this->get_logger(),
                        "Drive: Increase Speed -> %d", drive_speed);
        }

        // B button: Speed 4
        else if (msg->buttons[3] == 1) {
            drive_speed = 4;
            RCLCPP_INFO(this->get_logger(), "Drive: Right");
        }

        // Y button: Speed 5
        else if (msg->buttons[1] == 1) {
            drive_speed = 5;
            RCLCPP_INFO(this->get_logger(), "Drive: Left");
        }
	else if(msg->axes[7] == 1) {
            drive_speed = 10;
            RCLCPP_INFO(this->get_logger(), "Lifting...");
        }
       else if(msg->axes[7] == -1) {
            drive_speed = 9;
            RCLCPP_INFO(this->get_logger(), "Going Down...");
        }
        else if(msg->axes[6] == 1) {
            drive_speed = 7;
            RCLCPP_INFO(this->get_logger(), "Jaw Open...");
        }
        else if(msg->axes[6] == -1) {
            drive_speed = 8;
            RCLCPP_INFO(this->get_logger(), "Jaw Close...");
        }
        else {
            drive_speed = 0;  // default idle
        }

        // publish speed only
        drive_speed_msg.data = std::abs(drive_speed);
        drive_speed_publisher_->publish(drive_speed_msg);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr drive_speed_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyControl>();
    RCLCPP_INFO(node->get_logger(),
                "Joystick rover control node initialized.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

