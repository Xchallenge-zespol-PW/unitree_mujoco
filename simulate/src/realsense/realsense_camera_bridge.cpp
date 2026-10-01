#include "realsense_camera_bridge.hpp"

#include <chrono>
#include <iostream>
#include <utility>

#include <GLFW/glfw3.h>

namespace xchallenge
{

    RealSenseCameraBridge::RealSenseCameraBridge(std::string color_camera_name, std::string depth_camera_name, int width, int height, int fps, rclcpp::Node::SharedPtr ros_node)
        : color_camera_name_(std::move(color_camera_name)), depth_camera_name_(std::move(depth_camera_name)), width_(width), height_(height), fps_(fps), ros_node_(std::move(ros_node))
    {
    }

    RealSenseCameraBridge::~RealSenseCameraBridge()
    {
        Stop();
    }

    bool RealSenseCameraBridge::Bind(mjModel* model, mjData* data, std::recursive_mutex* simulation_mutex)
    {
        if (model == nullptr || data == nullptr || simulation_mutex == nullptr || ros_node_ == nullptr) {
            std::cerr << "[RealSenseCameraBridge] Invalid binding state." << std::endl;
            return false;
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex_);

            if (bound_) {
                return false;
            }
        }

        const int color_camera_id = mj_name2id(model, mjOBJ_CAMERA, color_camera_name_.c_str());
        const int depth_camera_id = mj_name2id(model, mjOBJ_CAMERA, depth_camera_name_.c_str());

        if (color_camera_id < 0 || depth_camera_id < 0) {
            return false;
        }

        const double fovy_rad = model->cam_fovy[color_camera_id] * mjPI / 180.0;
        const double focal_length_px = static_cast<double>(height_) / (2.0 * std::tan(fovy_rad / 2.0));

        CameraIntrinsics intrinsics;

        intrinsics.width = width_;
        intrinsics.height = height_;

        intrinsics.fx = focal_length_px;
        intrinsics.fy = focal_length_px;

        intrinsics.cx = (static_cast<double>(width_) - 1.0) / 2.0;
        intrinsics.cy = (static_cast<double>(height_) - 1.0) / 2.0;

        auto ros_publisher = std::make_unique<RosCameraPublisher>(ros_node_, intrinsics, "camera_color_optical_frame", 0.001f);

        if (!ros_publisher->IsReady()) {
            return false;
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex_);

            model_ = model;
            data_ = data;
            simulation_mutex_ = simulation_mutex;

            color_camera_id_ = color_camera_id;
            depth_camera_id_ = depth_camera_id;

            ros_publisher_ = std::move(ros_publisher);

            bound_ = true;
        }

        state_cv_.notify_all();

        return true;
    }    

    void RealSenseCameraBridge::Unbind()
    {
        if (running_.load()) {
            return;
        }

        std::lock_guard<std::mutex> lock(state_mutex_);

        ros_publisher_.reset();

        model_ = nullptr;
        data_ = nullptr;
        simulation_mutex_ = nullptr;

        color_camera_id_ = -1;
        depth_camera_id_ = -1;

        bound_ = false;
    }

    bool RealSenseCameraBridge::Start()
    {
        if (running_.load()) {
            return true;
        }

        if (camera_thread_.joinable() || gl_context_window_ != nullptr) {
            return false;
        }

        if (width_ <= 0 || height_ <= 0 || fps_ <= 0) {
            std::cerr << "[RealSenseCameraBridge] Invalid camera configuration." << std::endl;
            return false;
        }

        glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);

        gl_context_window_ = glfwCreateWindow(64, 64, "xchallenge_rgbd_context_test", nullptr, nullptr);

        if (gl_context_window_ == nullptr) {
            std::cerr << "[RealSenseCameraBridge] Failed to create OpenGL context." << std::endl;
            return false;
        }

        running_.store(true);

        try {
            camera_thread_ = std::thread(&RealSenseCameraBridge::CameraLoop, this);
        }
        catch (...) {
            running_.store(false);

            glfwDestroyWindow(gl_context_window_);
            gl_context_window_ = nullptr;

            throw;
        }

        return true;
    }

    void RealSenseCameraBridge::Stop()
    {
        running_.store(false);

        state_cv_.notify_all();

        if (camera_thread_.joinable()) {
            camera_thread_.join();
        }

        if (gl_context_window_ != nullptr) {
            glfwDestroyWindow(gl_context_window_);
            gl_context_window_ = nullptr;
        }
    }

    bool RealSenseCameraBridge::IsBound() const
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        return bound_;
    }

    bool RealSenseCameraBridge::IsRunning() const
    {
        return running_.load();
    }

    void RealSenseCameraBridge::CameraLoop()
    {
        glfwMakeContextCurrent(gl_context_window_);

        mjModel* model = nullptr;
        mjData* data = nullptr;
        std::recursive_mutex* simulation_mutex = nullptr;
        int color_camera_id = -1;

        {
            std::unique_lock<std::mutex> lock(state_mutex_);

            state_cv_.wait(lock, [this]() {
                return bound_ || !running_.load();
            });

            if (!running_.load()) {
                glfwMakeContextCurrent(nullptr);
                return;
            }

            model = model_;
            data = data_;
            simulation_mutex = simulation_mutex_;
            color_camera_id = color_camera_id_;
        }

        renderer_ = std::make_unique<MujocoRgbdRenderer>(model, data, color_camera_id, width_, height_);

        if (!renderer_->Initialize()) {
            renderer_.reset();

            running_.store(false);

            glfwMakeContextCurrent(nullptr);
            return;
        }

        const auto frame_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / static_cast<double>(fps_)));

        auto next_frame_time = std::chrono::steady_clock::now();

        using Clock = std::chrono::steady_clock;

        std::size_t diagnostic_frame_count = 0;

        while (running_.load() && rclcpp::ok()) {
            next_frame_time += frame_period;

            const auto frame_start = Clock::now();

            const auto lock_start = Clock::now();

            bool scene_updated = false;

            std::chrono::steady_clock::time_point update_start;
            std::chrono::steady_clock::time_point update_end;

            {
                std::lock_guard<std::recursive_mutex> simulation_lock(*simulation_mutex);

                update_start = Clock::now();

                scene_updated = renderer_->UpdateScene();

                update_end = Clock::now();
            }

            const auto lock_end = Clock::now();

            if (!scene_updated) {
                break;
            }

            const auto render_start = Clock::now();

            if (!renderer_->RenderRgbd()) {
                break;
            }

            const auto render_end = Clock::now();

            const RgbdFrame& frame = renderer_->Frame();

            const auto publish_start = Clock::now();

            if (!ros_publisher_->Publish(frame)) {
                break;
            }

            const auto publish_end = Clock::now();

            ++diagnostic_frame_count;

            if (diagnostic_frame_count % 10 == 0) {
                const double lock_total_ms = std::chrono::duration<double, std::milli>(lock_end - lock_start).count();
                const double update_ms = std::chrono::duration<double, std::milli>(update_end - update_start).count();
                const double render_ms = std::chrono::duration<double, std::milli>(render_end - render_start).count();
                const double publish_ms = std::chrono::duration<double, std::milli>(publish_end - publish_start).count();
                const double total_ms = std::chrono::duration<double, std::milli>(publish_end - frame_start).count();

                std::cout << "[CameraTiming] lock+update=" << lock_total_ms
                        << " ms, update=" << update_ms
                        << " ms, render=" << render_ms
                        << " ms, publish=" << publish_ms
                        << " ms, total=" << total_ms
                        << " ms" << std::endl;
            }

            const auto now = Clock::now();

            if (next_frame_time < now) {
                next_frame_time = now;
            }

            std::this_thread::sleep_until(next_frame_time);
        }

        if (renderer_) {
            renderer_->Shutdown();
            renderer_.reset();
        }
        
        glfwMakeContextCurrent(nullptr);

        running_.store(false);
    }
}  