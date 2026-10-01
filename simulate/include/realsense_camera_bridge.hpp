#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>

#include "mujoco_rgbd_renderer.hpp"
#include "ros_camera_publisher.hpp"

struct GLFWwindow;

namespace xchallenge
{

    class RealSenseCameraBridge
    {
    public:
        RealSenseCameraBridge(std::string color_camera_name, std::string depth_camera_name, int width, int height, int fps, rclcpp::Node::SharedPtr ros_node);
        ~RealSenseCameraBridge();

        RealSenseCameraBridge(const RealSenseCameraBridge&) = delete;
        RealSenseCameraBridge& operator=(const RealSenseCameraBridge&) = delete;

        bool Bind(mjModel* model, mjData* data, std::recursive_mutex* simulation_mutex);
        void Unbind();

        bool Start();
        void Stop();

        bool IsBound() const;
        bool IsRunning() const;

    private:
        void CameraLoop();

    private:
        std::string color_camera_name_;
        std::string depth_camera_name_;

        int width_ = 0;
        int height_ = 0;
        int fps_ = 0;

        rclcpp::Node::SharedPtr ros_node_;

        mjModel* model_ = nullptr;
        mjData* data_ = nullptr;

        std::recursive_mutex* simulation_mutex_ = nullptr;

        int color_camera_id_ = -1;
        int depth_camera_id_ = -1;

        std::unique_ptr<MujocoRgbdRenderer> renderer_;
        std::unique_ptr<RosCameraPublisher> ros_publisher_;

        GLFWwindow* gl_context_window_ = nullptr;

        std::thread camera_thread_;

        std::atomic<bool> running_{false};

        bool bound_ = false;

        mutable std::mutex state_mutex_;
        std::condition_variable state_cv_;
    };

}  