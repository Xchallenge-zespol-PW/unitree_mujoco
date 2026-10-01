#pragma once

#include <cstdint>
#include <vector>

#include <mujoco/mujoco.h>

#include "realsense_camera_types.hpp"

namespace xchallenge
{

    class MujocoRgbdRenderer
    {
    private:
        struct CameraResolution
        {
            int width = 0;
            int height = 0;
        };

    public:
        MujocoRgbdRenderer(const mjModel* model, mjData* data, int color_camera_id, int width, int height);
        ~MujocoRgbdRenderer();

        MujocoRgbdRenderer(const MujocoRgbdRenderer&) = delete;
        MujocoRgbdRenderer& operator=(const MujocoRgbdRenderer&) = delete;

        bool Initialize();
        bool UpdateScene();
        bool RenderRgbd();

        const RgbdFrame& Frame() const;

        bool IsInitialized() const;

        void Shutdown();

    private:
        void ConvertAndFlipFrame();

    private:
        bool initialized_ = false;
        bool scene_created_ = false;
        bool context_created_ = false;

        const mjModel* model_ = nullptr;
        mjData* data_ = nullptr;

        int color_camera_id_ = -1;

        CameraResolution resolution_;

        std::vector<std::uint8_t> raw_rgb_;
        std::vector<float> raw_depth_;

        RgbdFrame frame_;

        mjvCamera camera_{};
        mjvOption option_{};
        mjvScene scene_{};
        mjrContext context_{};
};

}  