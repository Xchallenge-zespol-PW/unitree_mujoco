#pragma once

#include <memory>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "mujoco_odometry.hpp"

namespace xchallenge {

    class RosOdometryPublisher {
    public:
        RosOdometryPublisher(rclcpp::Node::SharedPtr node, const OdometryConfig& config);
        void Publish(const OdometryFrame& frame);

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        nav_msgs::msg::Odometry message_;
        geometry_msgs::msg::TransformStamped transform_;
    };

}
