#include "mujoco_rgbd_renderer.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
#include <chrono>
#include <GLFW/glfw3.h>

namespace xchallenge
{

    MujocoRgbdRenderer::MujocoRgbdRenderer(const mjModel* model, mjData* data, int color_camera_id, int width, int height)
        : model_(model), data_(data), color_camera_id_(color_camera_id), resolution_{width, height}
    {
        mjv_defaultCamera(&camera_);
        mjv_defaultOption(&option_);
        mjv_defaultScene(&scene_);
        mjr_defaultContext(&context_);
    }

    MujocoRgbdRenderer::~MujocoRgbdRenderer()
    {
        Shutdown();
    }

    bool MujocoRgbdRenderer::Initialize()
    {
        if (initialized_) {
            return true;
        }

        if (model_ == nullptr || data_ == nullptr) {
            std::cerr << "[MujocoRgbdRenderer] Invalid model or data pointer." << std::endl;
            return false;
        }

        if (color_camera_id_ < 0 || color_camera_id_ >= model_->ncam) {
            std::cerr << "[MujocoRgbdRenderer] Invalid camera ID." << std::endl;
            return false;
        }

        if (resolution_.width <= 0 || resolution_.height <= 0) {
            std::cerr << "[MujocoRgbdRenderer] Invalid camera resolution." << std::endl;
            return false;
        }

        camera_.type = mjCAMERA_FIXED;
        camera_.fixedcamid = color_camera_id_;

        const int max_geometries = std::max(2000, model_->ngeom * 2);

        mjv_makeScene(model_, &scene_, max_geometries);
        scene_created_ = true;

        scene_.flags[mjRND_SHADOW] = 0;
        scene_.flags[mjRND_REFLECTION] = 0;

        std::cout << "[MuJoCo] offsamples=" << model_->vis.quality.offsamples
          << ", shadowsize=" << model_->vis.quality.shadowsize
          << ", offwidth=" << model_->vis.global.offwidth
          << ", offheight=" << model_->vis.global.offheight
          << std::endl;

        mjr_makeContext(model_, &context_, mjFONTSCALE_100);
        context_created_ = true;

        const char* gl_vendor = reinterpret_cast<const char*>(glGetString(GL_VENDOR));
        const char* gl_renderer = reinterpret_cast<const char*>(glGetString(GL_RENDERER));
        const char* gl_version = reinterpret_cast<const char*>(glGetString(GL_VERSION));

        std::cout << "[OpenGL] vendor=" << (gl_vendor != nullptr ? gl_vendor : "unknown") << std::endl;
        std::cout << "[OpenGL] renderer=" << (gl_renderer != nullptr ? gl_renderer : "unknown") << std::endl;
        std::cout << "[OpenGL] version=" << (gl_version != nullptr ? gl_version : "unknown") << std::endl;

        mjr_resizeOffscreen(resolution_.width, resolution_.height, &context_);
        mjr_setBuffer(mjFB_OFFSCREEN, &context_);

        if (context_.currentBuffer != mjFB_OFFSCREEN) {
            std::cerr << "[MujocoRgbdRenderer] Offscreen framebuffer is unavailable." << std::endl;
            Shutdown();
            return false;
        }

        context_.readDepthMap = mjDEPTH_ZERONEAR;

        const std::size_t pixel_count = static_cast<std::size_t>(resolution_.width) * static_cast<std::size_t>(resolution_.height);

        raw_rgb_.resize(pixel_count * 3);
        raw_depth_.resize(pixel_count);

        frame_.width = resolution_.width;
        frame_.height = resolution_.height;
        frame_.simulation_time_s = 0.0;

        frame_.rgb.resize(pixel_count * 3);
        frame_.depth_m.resize(pixel_count);

        initialized_ = true;

        return true;
    }

    bool MujocoRgbdRenderer::UpdateScene()
    {
        if (!initialized_) {
            return false;
        }

        frame_.simulation_time_s = data_->time;

        mjv_updateScene(model_, data_, &option_, nullptr, &camera_, mjCAT_ALL, &scene_);

        return true;
    }

    bool MujocoRgbdRenderer::RenderRgbd()
    {
        if (!initialized_) {
            return false;
        }

        using Clock = std::chrono::steady_clock;

        static std::size_t diagnostic_frame_count = 0;

        const mjrRect viewport{0, 0, resolution_.width, resolution_.height};

        mjr_setBuffer(mjFB_OFFSCREEN, &context_);

        const auto render_start = Clock::now();

        mjr_render(viewport, &scene_, &context_);

        const auto render_end = Clock::now();

        const auto finish_start = Clock::now();

        glFinish();

        const auto finish_end = Clock::now();

        const auto read_start = Clock::now();

        mjr_readPixels(raw_rgb_.data(), raw_depth_.data(), viewport, &context_);

        const auto read_end = Clock::now();

        if (mjr_getError() != 0) {
            std::cerr << "[MujocoRgbdRenderer] OpenGL rendering error." << std::endl;
            return false;
        }

        const auto convert_start = Clock::now();

        ConvertAndFlipFrame();

        const auto convert_end = Clock::now();

        ++diagnostic_frame_count;

        if (diagnostic_frame_count % 10 == 0) {
            const double render_ms = std::chrono::duration<double, std::milli>(render_end - render_start).count();
            const double finish_ms = std::chrono::duration<double, std::milli>(finish_end - finish_start).count();
            const double read_ms = std::chrono::duration<double, std::milli>(read_end - read_start).count();
            const double convert_ms = std::chrono::duration<double, std::milli>(convert_end - convert_start).count();

            std::cout << "[RenderTiming] mjr_render=" << render_ms
                    << " ms, glFinish=" << finish_ms
                    << " ms, mjr_readPixels=" << read_ms
                    << " ms, convert=" << convert_ms
                    << " ms" << std::endl;
        }

        return true;
    }

    void MujocoRgbdRenderer::ConvertAndFlipFrame()
    {
        const int width = resolution_.width;
        const int height = resolution_.height;

        const float near_plane = static_cast<float>(model_->vis.map.znear * model_->stat.extent);
        const float far_plane = static_cast<float>(model_->vis.map.zfar * model_->stat.extent);

        for (int y = 0; y < height; ++y) {
            const int source_y = height - 1 - y;

            for (int x = 0; x < width; ++x) {
                const std::size_t source_index = static_cast<std::size_t>(source_y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x);
                const std::size_t destination_index = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x);

                frame_.rgb[3 * destination_index + 0] = raw_rgb_[3 * source_index + 0];
                frame_.rgb[3 * destination_index + 1] = raw_rgb_[3 * source_index + 1];
                frame_.rgb[3 * destination_index + 2] = raw_rgb_[3 * source_index + 2];

                const float depth_buffer_value = std::clamp(raw_depth_[source_index], 0.0f, 1.0f);

                if (depth_buffer_value >= 1.0f - 1e-6f) {
                    frame_.depth_m[destination_index] = std::numeric_limits<float>::quiet_NaN();
                } else {
                    frame_.depth_m[destination_index] = near_plane / (1.0f - depth_buffer_value * (1.0f - near_plane / far_plane));
                }
            }
        }
    }

    const RgbdFrame& MujocoRgbdRenderer::Frame() const
    {
        return frame_;
    }

    bool MujocoRgbdRenderer::IsInitialized() const
    {
        return initialized_;
    }

    void MujocoRgbdRenderer::Shutdown()
    {
        if (context_created_) {
            mjr_freeContext(&context_);
            context_created_ = false;
        }

        if (scene_created_) {
            mjv_freeScene(&scene_);
            scene_created_ = false;
        }

        raw_rgb_.clear();
        raw_depth_.clear();

        frame_.rgb.clear();
        frame_.depth_m.clear();

        frame_.width = 0;
        frame_.height = 0;
        frame_.simulation_time_s = 0.0;

        initialized_ = false;
    }

} 