#pragma once

#include <cstddef>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "mujoco_lidar3d.hpp"

namespace xchallenge {

    class RosLidarPublisher {
    public:
        RosLidarPublisher(rclcpp::Node::SharedPtr node, const Lidar3dConfig& config);
        void Publish(const Lidar3dFrame& frame);

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
        sensor_msgs::msg::PointCloud2 message_;
        std::size_t frame_count_ = 0;
    };

} 
