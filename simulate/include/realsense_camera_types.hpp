#pragma once

#include <cstdint>
#include <vector>

namespace xchallenge
{

    struct CameraIntrinsics
    {
        int width = 0;
        int height = 0;

        double fx = 0.0;
        double fy = 0.0;

        double cx = 0.0;
        double cy = 0.0;
    };

    struct RgbdFrame
    {
        int width = 0;
        int height = 0;

        double simulation_time_s = 0.0;

        std::vector<std::uint8_t> rgb;
        std::vector<float> depth_m;
    };

} 