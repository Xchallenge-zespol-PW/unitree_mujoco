#pragma once

#include <array>
#include <limits>
#include <string>

#include <mujoco/mujoco.h>

namespace xchallenge {

    struct OdometryConfig {
        bool enabled = false;
        std::string body_name = "pelvis";
        std::string frame_id = "odom";
        std::string child_frame_id = "pelvis";
        std::string topic = "/odom";
        double rate_hz = 50.0;
        bool publish_tf = true;
    };

    struct OdometryFrame {
        double simulation_time_s = 0.0;
        // Body origin in the odom frame, metres. Orientation as w, x, y, z.
        std::array<double, 3> position{};
        std::array<double, 4> orientation{1.0, 0.0, 0.0, 0.0};
        // Expressed in the body (child) frame, as nav_msgs/Odometry requires.
        std::array<double, 3> linear_velocity{};
        std::array<double, 3> angular_velocity{};
    };

    // Ground-truth odometry of a free-floating body.
    // The odom frame is the body's x, y and yaw at the first capture (and again after a reset);
    // z, roll and pitch stay absolute, so the body keeps its true height and tilt.
    // Single-threaded: construct, capture and destroy on the physics thread.
    // model is borrowed; its owner must keep it alive and must not reload it.
    // CaptureIfDue requires the simulation mutex; no ROS.
    class MujocoOdometry {
    public:
        MujocoOdometry(const mjModel* model, OdometryConfig config);
        bool CaptureIfDue(const mjData* data);
        const OdometryFrame& Frame() const { return frame_; }

    private:
        const mjModel* model_;
        OdometryConfig config_;
        int qpos_adr_ = -1;
        int qvel_adr_ = -1;
        bool origin_set_ = false;
        std::array<mjtNum, 3> origin_position_{};
        std::array<mjtNum, 4> origin_yaw_inverse_{1.0, 0.0, 0.0, 0.0};
        double last_observed_time_ = -1.0;
        double next_capture_time_ = -std::numeric_limits<double>::infinity();
        OdometryFrame frame_;
    };

}
