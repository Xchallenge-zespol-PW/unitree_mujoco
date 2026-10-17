#include "ros_camera_publisher.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

#include <sensor_msgs/image_encodings.hpp>

namespace xchallenge
{

    RosCameraPublisher::RosCameraPublisher(rclcpp::Node::SharedPtr node, CameraIntrinsics intrinsics, std::string color_optical_frame_id, float depth_scale_m)
        : intrinsics_(intrinsics), color_optical_frame_id_(std::move(color_optical_frame_id)), depth_scale_m_(depth_scale_m)
    {
        if (node == nullptr) {
            return;
        }

        if (intrinsics_.width <= 0 || intrinsics_.height <= 0 || intrinsics_.fx <= 0.0 || intrinsics_.fy <= 0.0) {
            return;
        }

        if (depth_scale_m_ <= 0.0f) {
            return;
        }

        const auto qos = rclcpp::SensorDataQoS();

        color_image_publisher_ = node->create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw", qos);
        color_camera_info_publisher_ = node->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/color/camera_info", qos);

        aligned_depth_publisher_ = node->create_publisher<sensor_msgs::msg::Image>("/camera/aligned_depth_to_color/image_raw", qos);
        aligned_depth_camera_info_publisher_ = node->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/aligned_depth_to_color/camera_info", qos);

        ready_ = true;
    }

    bool RosCameraPublisher::IsReady() const
    {
        return ready_;
    }

    bool RosCameraPublisher::Publish(const RgbdFrame& frame)
    {
        if (!ready_) {
            return false;
        }

        if (frame.width != intrinsics_.width || frame.height != intrinsics_.height) {
            return false;
        }

        const std::size_t pixel_count = static_cast<std::size_t>(frame.width) * static_cast<std::size_t>(frame.height);

        if (frame.rgb.size() != pixel_count * 3 || frame.depth_m.size() != pixel_count) {
            return false;
        }

        const builtin_interfaces::msg::Time stamp = ToRosStamp(frame.simulation_time_s);

        sensor_msgs::msg::Image color_image = CreateColorImage(frame, stamp);
        sensor_msgs::msg::Image depth_image = CreateDepthImage(frame, stamp);

        sensor_msgs::msg::CameraInfo color_camera_info = CreateCameraInfo(stamp);
        sensor_msgs::msg::CameraInfo depth_camera_info = CreateCameraInfo(stamp);

        color_image_publisher_->publish(std::move(color_image));
        color_camera_info_publisher_->publish(std::move(color_camera_info));

        aligned_depth_publisher_->publish(std::move(depth_image));
        aligned_depth_camera_info_publisher_->publish(std::move(depth_camera_info));

        return true;
    }

    builtin_interfaces::msg::Time RosCameraPublisher::ToRosStamp(double simulation_time_s) const
    {
        builtin_interfaces::msg::Time stamp;

        if (!std::isfinite(simulation_time_s) || simulation_time_s < 0.0) {
            return stamp;
        }

        constexpr std::int64_t nanoseconds_per_second = 1'000'000'000LL;

        const std::int64_t total_nanoseconds = static_cast<std::int64_t>(std::llround(simulation_time_s * static_cast<double>(nanoseconds_per_second)));

        stamp.sec = static_cast<std::int32_t>(total_nanoseconds / nanoseconds_per_second);
        stamp.nanosec = static_cast<std::uint32_t>(total_nanoseconds % nanoseconds_per_second);

        return stamp;
    }

    sensor_msgs::msg::CameraInfo RosCameraPublisher::CreateCameraInfo(const builtin_interfaces::msg::Time& stamp) const
    {
        sensor_msgs::msg::CameraInfo message;

        message.header.stamp = stamp;
        message.header.frame_id = color_optical_frame_id_;

        message.width = static_cast<std::uint32_t>(intrinsics_.width);
        message.height = static_cast<std::uint32_t>(intrinsics_.height);

        message.distortion_model = "plumb_bob";
        message.d = {0.0, 0.0, 0.0, 0.0, 0.0};

        message.k = {
            intrinsics_.fx, 0.0, intrinsics_.cx,
            0.0, intrinsics_.fy, intrinsics_.cy,
            0.0, 0.0, 1.0
        };

        message.r = {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };

        message.p = {
            intrinsics_.fx, 0.0, intrinsics_.cx, 0.0,
            0.0, intrinsics_.fy, intrinsics_.cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        };

        return message;
    }

    sensor_msgs::msg::Image RosCameraPublisher::CreateColorImage(const RgbdFrame& frame, const builtin_interfaces::msg::Time& stamp) const
    {
        sensor_msgs::msg::Image message;

        message.header.stamp = stamp;
        message.header.frame_id = color_optical_frame_id_;

        message.width = static_cast<std::uint32_t>(frame.width);
        message.height = static_cast<std::uint32_t>(frame.height);

        message.encoding = sensor_msgs::image_encodings::RGB8;
        message.is_bigendian = false;
        message.step = static_cast<std::uint32_t>(frame.width * 3);

        message.data = frame.rgb;

        return message;
    }

    sensor_msgs::msg::Image RosCameraPublisher::CreateDepthImage(const RgbdFrame& frame, const builtin_interfaces::msg::Time& stamp) const
    {
        sensor_msgs::msg::Image message;

        message.header.stamp = stamp;
        message.header.frame_id = color_optical_frame_id_;

        message.width = static_cast<std::uint32_t>(frame.width);
        message.height = static_cast<std::uint32_t>(frame.height);

        message.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        message.is_bigendian = false;
        message.step = static_cast<std::uint32_t>(frame.width * sizeof(std::uint16_t));

        message.data.resize(frame.depth_m.size() * sizeof(std::uint16_t));

        for (std::size_t i = 0; i < frame.depth_m.size(); ++i) {
            const float depth_m = frame.depth_m[i];

            std::uint16_t depth_value = 0;

            if (std::isfinite(depth_m) && depth_m > 0.0f) {
                const double scaled_depth = static_cast<double>(depth_m) / static_cast<double>(depth_scale_m_);

                if (scaled_depth >= 1.0 && scaled_depth <= static_cast<double>(std::numeric_limits<std::uint16_t>::max())) {
                    depth_value = static_cast<std::uint16_t>(std::lround(scaled_depth));
                }
            }

            message.data[2 * i] = static_cast<std::uint8_t>(depth_value & 0xFF);
            message.data[2 * i + 1] = static_cast<std::uint8_t>((depth_value >> 8) & 0xFF);
        }

        return message;
    }

}  