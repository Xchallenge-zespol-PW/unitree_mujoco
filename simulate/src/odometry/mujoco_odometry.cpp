#include "mujoco_odometry.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace xchallenge {

MujocoOdometry::MujocoOdometry(const mjModel* model, OdometryConfig config)
    : model_(model), config_(std::move(config)) {
    if (!model_ || !std::isfinite(config_.rate_hz) || config_.rate_hz <= 0.0 || config_.rate_hz > 1000.0) {
        throw std::invalid_argument("Invalid odometry model or rate");
    }
    const int body_id = mj_name2id(model_, mjOBJ_BODY, config_.body_name.c_str());
    if (body_id < 0) {
        throw std::invalid_argument("Missing MuJoCo odometry body: " + config_.body_name);
    }
    // A free joint gives the body pose and velocity directly from qpos/qvel.
    const int joint_id = model_->body_jntadr[body_id];
    if (model_->body_jntnum[body_id] < 1 || model_->jnt_type[joint_id] != mjJNT_FREE) {
        throw std::invalid_argument("Odometry body has no free joint: " + config_.body_name);
    }
    qpos_adr_ = model_->jnt_qposadr[joint_id];
    qvel_adr_ = model_->jnt_dofadr[joint_id];
}

bool MujocoOdometry::CaptureIfDue(const mjData* data) {
    if (!data || !std::isfinite(data->time) || data->time < 0.0) {
        throw std::invalid_argument("Invalid odometry mjData or simulation time");
    }
    const double stamp = data->time;
    if (stamp < last_observed_time_) {
        next_capture_time_ = stamp;  // reset/rewind: resume immediately
        origin_set_ = false;         // and restart odometry from the new pose
    }
    last_observed_time_ = stamp;
    if (stamp + 1e-9 < next_capture_time_) {
        return false;
    }

    const mjtNum* position = data->qpos + qpos_adr_;
    const mjtNum* orientation = data->qpos + qpos_adr_ + 3;
    // Free joint: linear velocity in the world frame, angular velocity already in the body frame.
    const mjtNum* linear_velocity_world = data->qvel + qvel_adr_;
    const mjtNum* angular_velocity_body = data->qvel + qvel_adr_ + 3;

    if (!origin_set_) {
        const mjtNum yaw = std::atan2(2.0 * (orientation[0] * orientation[3] + orientation[1] * orientation[2]),
                                      1.0 - 2.0 * (orientation[2] * orientation[2] + orientation[3] * orientation[3]));
        origin_position_ = {position[0], position[1], 0.0};
        origin_yaw_inverse_ = {std::cos(yaw / 2.0), 0.0, 0.0, -std::sin(yaw / 2.0)};
        origin_set_ = true;
    }

    mjtNum offset[3];
    mjtNum odom_position[3];
    mjtNum odom_orientation[4];
    mju_sub3(offset, position, origin_position_.data());
    mju_rotVecQuat(odom_position, offset, origin_yaw_inverse_.data());
    mju_mulQuat(odom_orientation, origin_yaw_inverse_.data(), orientation);
    mju_normalize4(odom_orientation);

    mjtNum body_inverse[4];
    mjtNum linear_velocity_body[3];
    mju_negQuat(body_inverse, orientation);
    mju_rotVecQuat(linear_velocity_body, linear_velocity_world, body_inverse);

    frame_.simulation_time_s = stamp;
    for (int i = 0; i < 3; ++i) {
        frame_.position[i] = odom_position[i];
        frame_.linear_velocity[i] = linear_velocity_body[i];
        frame_.angular_velocity[i] = angular_velocity_body[i];
    }
    for (int i = 0; i < 4; ++i) {
        frame_.orientation[i] = odom_orientation[i];
    }

    next_capture_time_ = stamp + 1.0 / config_.rate_hz;
    return true;
}

}
