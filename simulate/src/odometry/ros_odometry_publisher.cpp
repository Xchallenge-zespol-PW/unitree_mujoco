#include "ros_odometry_publisher.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

namespace xchallenge {

    RosOdometryPublisher::RosOdometryPublisher(rclcpp::Node::SharedPtr node, const OdometryConfig& config)
        : node_(std::move(node)) {
        if (!node_ || config.frame_id.empty() || config.child_frame_id.empty() || config.topic.empty()) {
            throw std::invalid_argument("Invalid odometry ROS node, frames or topic");
        }
        // Reliable, so default-QoS subscribers such as nav2 can connect.
        publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(config.topic, rclcpp::QoS(10));
        if (config.publish_tf) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
        }
        message_.header.frame_id = config.frame_id;
        message_.child_frame_id = config.child_frame_id;
        transform_.header.frame_id = config.frame_id;
        transform_.child_frame_id = config.child_frame_id;
    }

    void RosOdometryPublisher::Publish(const OdometryFrame& frame) {
        const auto context = node_->get_node_base_interface()->get_context();
        if (!rclcpp::ok(context)) {
            return;
        }
        if (!std::isfinite(frame.simulation_time_s) || frame.simulation_time_s < 0.0 || frame.simulation_time_s >= std::numeric_limits<std::int32_t>::max()) {
            throw std::runtime_error("Invalid odometry timestamp");
        }
        const auto ns = static_cast<std::int64_t>(std::llround(frame.simulation_time_s * 1000000000.0));
        message_.header.stamp.sec = static_cast<std::int32_t>(ns / 1000000000LL);
        message_.header.stamp.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);

        auto& pose = message_.pose.pose;
        pose.position.x = frame.position[0];
        pose.position.y = frame.position[1];
        pose.position.z = frame.position[2];
        pose.orientation.w = frame.orientation[0];
        pose.orientation.x = frame.orientation[1];
        pose.orientation.y = frame.orientation[2];
        pose.orientation.z = frame.orientation[3];

        auto& twist = message_.twist.twist;
        twist.linear.x = frame.linear_velocity[0];
        twist.linear.y = frame.linear_velocity[1];
        twist.linear.z = frame.linear_velocity[2];
        twist.angular.x = frame.angular_velocity[0];
        twist.angular.y = frame.angular_velocity[1];
        twist.angular.z = frame.angular_velocity[2];

        transform_.header.stamp = message_.header.stamp;
        transform_.transform.translation.x = pose.position.x;
        transform_.transform.translation.y = pose.position.y;
        transform_.transform.translation.z = pose.position.z;
        transform_.transform.rotation = pose.orientation;

        // One message with one stamp, so that lookups between the pelvis and the bodies find a common time.
        transforms_.assign(1, transform_);
        for (const BodyPose& body : frame.extra_bodies) {
            auto& transform = transforms_.emplace_back(transform_);
            transform.child_frame_id = body.name;
            transform.transform.translation.x = body.position[0];
            transform.transform.translation.y = body.position[1];
            transform.transform.translation.z = body.position[2];
            transform.transform.rotation.w = body.orientation[0];
            transform.transform.rotation.x = body.orientation[1];
            transform.transform.rotation.y = body.orientation[2];
            transform.transform.rotation.z = body.orientation[3];
        }

        try {
            publisher_->publish(message_);
            if (tf_broadcaster_) {
                tf_broadcaster_->sendTransform(transforms_);
            }
        } catch (const rclcpp::exceptions::RCLError&) {
            // SIGINT can shut down ROS between the check above and publish().
            if (rclcpp::ok(context)) {
                throw;
            }
        }
    }

}
