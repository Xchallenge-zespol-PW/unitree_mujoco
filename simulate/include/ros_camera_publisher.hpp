#pragma once

#include <memory>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "realsense_camera_types.hpp"

namespace xchallenge
{

    class RosCameraPublisher
    {
    public:
        RosCameraPublisher(rclcpp::Node::SharedPtr node, CameraIntrinsics intrinsics, std::string color_optical_frame_id, float depth_scale_m);

        bool IsReady() const;
        bool Publish(const RgbdFrame& frame);

    private:
        builtin_interfaces::msg::Time ToRosStamp(double simulation_time_s) const;
        sensor_msgs::msg::CameraInfo CreateCameraInfo(const builtin_interfaces::msg::Time& stamp) const;
        sensor_msgs::msg::Image CreateColorImage(const RgbdFrame& frame, const builtin_interfaces::msg::Time& stamp) const;
        sensor_msgs::msg::Image CreateDepthImage(const RgbdFrame& frame, const builtin_interfaces::msg::Time& stamp) const;

    private:
        CameraIntrinsics intrinsics_;

        std::string color_optical_frame_id_;

        float depth_scale_m_ = 0.001f;

        bool ready_ = false;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_image_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_camera_info_publisher_;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr aligned_depth_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr aligned_depth_camera_info_publisher_;
};

}  