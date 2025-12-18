#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class CmdVelRelay : public rclcpp::Node {
public:

    CmdVelRelay() : Node("cmd_vel_relay") {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&CmdVelRelay::cmd_vel_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mecanum_drive_controller/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Velocity relay node started");
    }

private:

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {

        auto stamped_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        stamped_msg->header.stamp = this->now();
        stamped_msg->header.frame_id = "base_link";
        stamped_msg->twist = *msg;

        publisher_->publish(std::move(stamped_msg));
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdVelRelay>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
