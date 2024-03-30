#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class TwistToTwistStampedNode : public rclcpp::Node {
public:
    TwistToTwistStampedNode() : Node("twist_to_twiststamped_node") {
        // Create a publisher for TwistStamped topic
        twist_stamped_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/servo_node/delta_twist_cmds", 10);

        // Create a subscriber for Twist topic
        twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/spacenav/twist", 10, std::bind(&TwistToTwistStampedNode::twistCallback, this, std::placeholders::_1));
    }

private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr twist_msg) {
        // Create a TwistStamped message
        auto twist_stamped_msg = geometry_msgs::msg::TwistStamped();

        // Populate the header
        twist_stamped_msg.header.stamp = this->now();
        twist_stamped_msg.header.frame_id = "panda_link0";

        // Copy the twist data
        twist_stamped_msg.twist = *twist_msg;

        // Publish the TwistStamped message
        twist_stamped_publisher_->publish(twist_stamped_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistToTwistStampedNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

