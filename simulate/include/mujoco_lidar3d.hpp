#pragma once

#include <array>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

namespace xchallenge {

    struct Lidar3dConfig {
        bool enabled = false;
        std::string site_name = "lidar_site";
        std::string frame_id = "lidar_frame";
        std::string topic = "/lidar/points";
        int horizontal_samples = 180;
        int vertical_samples = 16;
        double vertical_min_deg = -15.0;
        double vertical_max_deg = 15.0;
        double range_min_m = 0.1;
        double range_max_m = 20.0;
        double rate_hz = 5.0;
        bool exclude_mount_body = true;
        std::array<mjtByte, mjNGROUP> geom_groups = [] {
            std::array<mjtByte, mjNGROUP> groups{};
            groups[0] = 1;  // Ray-cast against collision geometry; ignore visual-only groups by default.
            return groups;
        }();
    };

    struct Lidar3dFrame {
        double simulation_time_s = 0.0;
        double capture_ms = 0.0;
        int width = 0;
        int height = 0;
        std::size_t valid_points = 0;
        // Row-major: elevation row, then azimuth column. XYZ in the site frame, metres.
        // No hit or out of range: three NaNs, keeping the organized cloud shape.
        std::vector<float> xyz;
    };

    // Single-threaded: construct, capture and destroy on the physics thread.
    // model is borrowed; its owner must keep it alive and must not reload it.
    // CaptureIfDue requires the simulation mutex; no OpenGL, ROS or worker thread.
    class MujocoLidar3d {
    public:
        MujocoLidar3d(const mjModel* model, Lidar3dConfig config);
        bool CaptureIfDue(mjData* data);
        const Lidar3dFrame& Frame() const { return frame_; }

    private:
        const mjModel* model_;
        Lidar3dConfig config_;
        int site_id_ = -1;
        int excluded_body_ = -1;
        double last_observed_time_ = -1.0;
        double next_capture_time_ = -std::numeric_limits<double>::infinity();
        std::vector<mjtNum> local_directions_;
        std::vector<mjtNum> world_directions_;
        std::vector<mjtNum> ranges_;
        std::vector<int> geom_ids_;
        Lidar3dFrame frame_;
    };

} 
