#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace catch2023_principal {
class catch_tf {
    public:
    catch_tf();
    ~catch_tf();
    
    private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_subscriber;
    rclcpp::Node::SharedPtr node;
};
}  // namespace catch2023_principal