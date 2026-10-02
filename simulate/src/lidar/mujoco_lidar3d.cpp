#include "mujoco_lidar3d.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace xchallenge {

MujocoLidar3d::MujocoLidar3d(const mjModel* model, Lidar3dConfig config)
    : model_(model), config_(std::move(config)) {
    const bool finite = std::isfinite(config_.vertical_min_deg) && std::isfinite(config_.vertical_max_deg) && std::isfinite(config_.range_min_m) && std::isfinite(config_.range_max_m) && std::isfinite(config_.rate_hz);
    if (!model_ || !finite || config_.horizontal_samples < 4 || config_.horizontal_samples > 2048 || config_.vertical_samples < 2 || config_.vertical_samples > 128 || config_.rate_hz <= 0.0 || config_.rate_hz > 100.0 || config_.range_min_m < 0.0 || config_.range_max_m <= config_.range_min_m || config_.vertical_min_deg <= -90.0 || config_.vertical_max_deg >= 90.0 || config_.vertical_min_deg >= config_.vertical_max_deg) {
        throw std::invalid_argument("Invalid LiDAR model, sample counts, angles, range or rate");
    }
    site_id_ = mj_name2id(model_, mjOBJ_SITE, config_.site_name.c_str());
    if (site_id_ < 0) {
        throw std::invalid_argument("Missing MuJoCo LiDAR site: " + config_.site_name);
    }
    // Never exclude world body 0: that would hide static walls and terrain.
    const int mount_body = model_->site_bodyid[site_id_];
    excluded_body_ = config_.exclude_mount_body && mount_body != 0 ? mount_body : -1;

    frame_.width = config_.horizontal_samples;
    frame_.height = config_.vertical_samples;
    const auto count = static_cast<std::size_t>(frame_.width) * frame_.height;
    local_directions_.resize(3 * count);
    world_directions_.resize(3 * count);
    ranges_.resize(count);
    geom_ids_.resize(count);
    frame_.xyz.resize(3 * count);
    for (int row = 0; row < frame_.height; ++row) {
        const double elevation = (config_.vertical_min_deg + (config_.vertical_max_deg - config_.vertical_min_deg) * row / (frame_.height - 1)) * mjPI / 180.0;
        for (int col = 0; col < frame_.width; ++col) {
            // [0, 2*pi): no duplicate 360-degree endpoint. +X forward, +Y left, +Z up.
            const double azimuth = 2.0 * mjPI * col / frame_.width;
            const auto offset = 3 * (static_cast<std::size_t>(row) * frame_.width + col);
            local_directions_[offset] = std::cos(elevation) * std::cos(azimuth);
            local_directions_[offset + 1] = std::cos(elevation) * std::sin(azimuth);
            local_directions_[offset + 2] = std::sin(elevation);
        }
    }
}

bool MujocoLidar3d::CaptureIfDue(mjData* data) {
    if (!data || !std::isfinite(data->time) || data->time < 0.0) {
        throw std::invalid_argument("Invalid LiDAR mjData or simulation time");
    }
    const double stamp = data->time;
    if (stamp < last_observed_time_) {
        next_capture_time_ = stamp;  // reset/rewind: resume immediately
    }
    last_observed_time_ = stamp;
    if (stamp + 1e-9 < next_capture_time_) {
        return false;
    }
    const auto start = std::chrono::steady_clock::now();
    // mj_step advances qpos; refresh transforms for exactly the stamped state.
    // Caller owns sim.mtx. mj_multiRay also uses mjData scratch stack.
    mj_kinematics(model_, data);
    const mjtNum* origin = data->site_xpos + 3 * site_id_;
    const mjtNum* rotation = data->site_xmat + 9 * site_id_;
    for (std::size_t i = 0; i < ranges_.size(); ++i) {
        mju_mulMatVec3(world_directions_.data() + 3 * i, rotation, local_directions_.data() + 3 * i);
    }
    mj_multiRay(model_, data, origin, world_directions_.data(), config_.geom_groups.data(), 1, excluded_body_, geom_ids_.data(), ranges_.data(), static_cast<int>(ranges_.size()), config_.range_max_m);

    const float invalid = std::numeric_limits<float>::quiet_NaN();
    frame_.valid_points = 0;
    for (std::size_t i = 0; i < ranges_.size(); ++i) {
        const mjtNum range = ranges_[i];
        const bool valid = geom_ids_[i] >= 0 && std::isfinite(range) && range >= config_.range_min_m && range <= config_.range_max_m;
        if (valid) {
            ++frame_.valid_points;
        }
        for (int axis = 0; axis < 3; ++axis) {
            frame_.xyz[3 * i + axis] = valid ? static_cast<float>(range * local_directions_[3 * i + axis]) : invalid;
        }
    }
    frame_.simulation_time_s = stamp;
    frame_.capture_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    next_capture_time_ = stamp + 1.0 / config_.rate_hz;
    return true;
}

} 
