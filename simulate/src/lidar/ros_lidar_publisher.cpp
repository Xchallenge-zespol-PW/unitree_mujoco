#include "ros_lidar_publisher.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

namespace xchallenge {

    RosLidarPublisher::RosLidarPublisher(rclcpp::Node::SharedPtr node, const Lidar3dConfig& config)
        : node_(std::move(node)) {
        if (!node_ || config.frame_id.empty() || config.topic.empty()) {
            throw std::invalid_argument("Invalid LiDAR ROS node, frame or topic");
        }
        publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(config.topic, rclcpp::SensorDataQoS());
        message_.header.frame_id = config.frame_id;
        message_.height = static_cast<std::uint32_t>(config.vertical_samples);
        message_.width = static_cast<std::uint32_t>(config.horizontal_samples);
        const std::uint16_t endian_probe = 1;
        message_.is_bigendian = *reinterpret_cast<const std::uint8_t*>(&endian_probe) == 0;
        message_.is_dense = false;
        static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559, "PointCloud2 requires IEEE754 float32");
        const char* names[] = {"x", "y", "z"};
        message_.fields.resize(3);
        for (std::uint32_t i = 0; i < 3; ++i) {
            message_.fields[i].name = names[i];
            message_.fields[i].offset = 4 * i;
            message_.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
            message_.fields[i].count = 1;
        }
        message_.point_step = 12;
        message_.row_step = message_.point_step * message_.width;
        message_.data.resize(static_cast<std::size_t>(message_.row_step) * message_.height);
    }

    void RosLidarPublisher::Publish(const Lidar3dFrame& frame) {
        const auto context = node_->get_node_base_interface()->get_context();
        if (!rclcpp::ok(context)) {
            return;
        }
        if (frame.width != static_cast<int>(message_.width) || frame.height != static_cast<int>(message_.height) || frame.xyz.size() * sizeof(float) != message_.data.size() || !std::isfinite(frame.simulation_time_s) || frame.simulation_time_s < 0.0 || frame.simulation_time_s >= std::numeric_limits<std::int32_t>::max()) {
            throw std::runtime_error("Invalid LiDAR frame dimensions or timestamp");
        }
        const auto ns = static_cast<std::int64_t>(std::llround(frame.simulation_time_s * 1000000000.0));
        message_.header.stamp.sec = static_cast<std::int32_t>(ns / 1000000000LL);
        message_.header.stamp.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
        std::memcpy(message_.data.data(), frame.xyz.data(), message_.data.size());
        try {
            publisher_->publish(message_);
        } catch (const rclcpp::exceptions::RCLError&) {
            // SIGINT can shut down ROS between the check above and publish().
            if (rclcpp::ok(context)) {
                throw;
            }
            return;
        }
        if (++frame_count_ % 20 == 0) {
            RCLCPP_INFO(node_->get_logger(), "[LidarTiming] raycast+pack=%.3f ms; valid=%zu/%zu; sim_time=%.3f", frame.capture_ms, frame.valid_points, frame.xyz.size() / 3, frame.simulation_time_s);
        }
    }

}
