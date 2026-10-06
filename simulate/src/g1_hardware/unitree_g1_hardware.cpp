#include "g1_hardware/unitree_g1_hardware.hpp"

#include <algorithm>
#include <stdexcept>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

namespace xchallenge::g1_hardware
{

    namespace
    {
        const std::unordered_map<std::string, std::size_t> kJointToDdsIndex = {
            {"left_hip_pitch_joint", unitree::robot::g1::LeftHipPitch},
            {"left_hip_roll_joint", unitree::robot::g1::LeftHipRoll},
            {"left_hip_yaw_joint", unitree::robot::g1::LeftHipYaw},
            {"left_knee_joint", unitree::robot::g1::LeftKnee},
            {"left_ankle_pitch_joint", unitree::robot::g1::LeftAnklePitch},
            {"left_ankle_roll_joint", unitree::robot::g1::LeftAnkleRoll},

            {"right_hip_pitch_joint", unitree::robot::g1::RightHipPitch},
            {"right_hip_roll_joint", unitree::robot::g1::RightHipRoll},
            {"right_hip_yaw_joint", unitree::robot::g1::RightHipYaw},
            {"right_knee_joint", unitree::robot::g1::RightKnee},
            {"right_ankle_pitch_joint", unitree::robot::g1::RightAnklePitch},
            {"right_ankle_roll_joint", unitree::robot::g1::RightAnkleRoll},

            {"waist_yaw_joint", unitree::robot::g1::WaistYaw},
            {"waist_roll_joint", unitree::robot::g1::WaistRoll},
            {"waist_pitch_joint", unitree::robot::g1::WaistPitch},

            {"left_shoulder_pitch_joint", unitree::robot::g1::LeftShoulderPitch},
            {"left_shoulder_roll_joint", unitree::robot::g1::LeftShoulderRoll},
            {"left_shoulder_yaw_joint", unitree::robot::g1::LeftShoulderYaw},
            {"left_elbow_joint", unitree::robot::g1::LeftElbow},
            {"left_wrist_roll_joint", unitree::robot::g1::LeftWristRoll},
            {"left_wrist_pitch_joint", unitree::robot::g1::LeftWristPitch},
            {"left_wrist_yaw_joint", unitree::robot::g1::LeftWristYaw},

            {"right_shoulder_pitch_joint", unitree::robot::g1::RightShoulderPitch},
            {"right_shoulder_roll_joint", unitree::robot::g1::RightShoulderRoll},
            {"right_shoulder_yaw_joint", unitree::robot::g1::RightShoulderYaw},
            {"right_elbow_joint", unitree::robot::g1::RightElbow},
            {"right_wrist_roll_joint", unitree::robot::g1::RightWristRoll},
            {"right_wrist_pitch_joint", unitree::robot::g1::RightWristPitch},
            {"right_wrist_yaw_joint", unitree::robot::g1::RightWristYaw},
        };
    }

    std::size_t UnitreeG1HardwareBase::DdsIndexForJoint(const std::string& joint_name) const
    {
        const auto iterator = kJointToDdsIndex.find(joint_name);

        if (iterator == kJointToDdsIndex.end()) {
            throw std::runtime_error("Unknown Unitree G1 joint: " + joint_name);
        }

        return iterator->second;
    }

    UnitreeG1HardwareBase::CallbackReturn UnitreeG1HardwareBase::on_init(const hardware_interface::HardwareInfo& info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }

        if (info_.hardware_parameters.count("domain_id") > 0) {
            domain_id_ = std::stoi(info_.hardware_parameters.at("domain_id"));
        }

        if (info_.hardware_parameters.count("network_interface") > 0) {
            network_interface_ = info_.hardware_parameters.at("network_interface");
        }

        if (info_.hardware_parameters.count("kp") > 0) {
            default_kp_ = std::stod(info_.hardware_parameters.at("kp"));
        }

        if (info_.hardware_parameters.count("kd") > 0) {
            default_kd_ = std::stod(info_.hardware_parameters.at("kd"));
        }

        const std::size_t joint_count = info_.joints.size();

        dds_indices_.resize(joint_count);

        state_position_.assign(joint_count, 0.0);
        state_velocity_.assign(joint_count, 0.0);
        state_effort_.assign(joint_count, 0.0);

        command_position_.assign(joint_count, 0.0);

        kp_.assign(joint_count, default_kp_);
        kd_.assign(joint_count, default_kd_);

        try
        {
            for (std::size_t i = 0; i < joint_count; ++i)
            {
                dds_indices_[i] = DdsIndexForJoint(info_.joints[i].name);

                const auto kp_iterator = info_.joints[i].parameters.find("kp");
                if (kp_iterator != info_.joints[i].parameters.end())
                {
                    kp_[i] = std::stod(kp_iterator->second);
                }

                const auto kd_iterator = info_.joints[i].parameters.find("kd");
                if (kd_iterator != info_.joints[i].parameters.end())
                {
                    kd_[i] = std::stod(kd_iterator->second);
                }
            }
        }
        catch (const std::exception& error) {
            RCLCPP_ERROR(rclcpp::get_logger("UnitreeG1Hardware"), "%s", error.what());
            return CallbackReturn::ERROR;
        }

        if (!ValidateJointSet()) {
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    bool UnitreeG1ArmHardware::ValidateJointSet() const
    {
        for (const std::size_t dds_index : dds_indices_)
        {
            if (dds_index < 12 || dds_index > 28)
            {
                RCLCPP_ERROR(rclcpp::get_logger("UnitreeG1ArmHardware"), "Joint index %zu is not allowed for ArmSdk hardware.", dds_index);
                return false;
            }
        }

        return true;
    }

    std::vector<hardware_interface::StateInterface> UnitreeG1HardwareBase::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> interfaces;
        interfaces.reserve(info_.joints.size() * 3);

        for (std::size_t i = 0; i < info_.joints.size(); ++i) {
            interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &state_position_[i]);
            interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_velocity_[i]);
            interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &state_effort_[i]);
        }

        return interfaces;
    }

    std::vector<hardware_interface::CommandInterface> UnitreeG1HardwareBase::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> interfaces;
        interfaces.reserve(info_.joints.size());

        for (std::size_t i = 0; i < info_.joints.size(); ++i) {
            interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &command_position_[i]);
        }

        return interfaces;
    }

    UnitreeG1HardwareBase::CallbackReturn UnitreeG1HardwareBase::on_configure(const rclcpp_lifecycle::State& previous_state)
    {
        try
        {
            unitree::robot::ChannelFactory::Instance()->Init(domain_id_, network_interface_);

            lowstate_ = std::make_shared<unitree::robot::g1::subscription::LowState>();
            lowstate_->wait_for_connection();

            if (!ConfigureCommandPublisher()) {
                return CallbackReturn::ERROR;
            }
        }
        catch (const std::exception& error) {
            RCLCPP_ERROR(rclcpp::get_logger("UnitreeG1Hardware"), "DDS configuration failed: %s", error.what());
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type UnitreeG1HardwareBase::read(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        if (lowstate_ == nullptr) {
            return hardware_interface::return_type::ERROR;
        }

        std::lock_guard<std::mutex> lock(lowstate_->mutex_);

        for (std::size_t i = 0; i < dds_indices_.size(); ++i) {
            const std::size_t dds_index = dds_indices_[i];

            state_position_[i] = lowstate_->msg_.motor_state()[dds_index].q();
            state_velocity_[i] = lowstate_->msg_.motor_state()[dds_index].dq();
            state_effort_[i] = lowstate_->msg_.motor_state()[dds_index].tau_est();
        }

        return hardware_interface::return_type::OK;
    }

    UnitreeG1HardwareBase::CallbackReturn UnitreeG1HardwareBase::on_activate(const rclcpp_lifecycle::State& previous_state)
    {
        if (lowstate_ == nullptr) {
            return CallbackReturn::ERROR;
        }

        {
            std::lock_guard<std::mutex> lock(lowstate_->mutex_);

            for (std::size_t i = 0; i < dds_indices_.size(); ++i) {
                command_position_[i] = lowstate_->msg_.motor_state()[dds_indices_[i]].q();
            }
        }

        active_ = true;

        return CallbackReturn::SUCCESS;
    }

    bool UnitreeG1SimHardware::ValidateJointSet() const
    {
        if (info_.joints.size() != 29) {
            RCLCPP_ERROR(rclcpp::get_logger("UnitreeG1SimHardware"), "Simulation hardware requires exactly 29 joints.");
            return false;
        }

        return true;
    }

    bool UnitreeG1SimHardware::ConfigureCommandPublisher()
    {
        lowcmd_ = std::make_unique<unitree::robot::g1::publisher::LowCmd>();
        return true;
    }

    hardware_interface::return_type UnitreeG1HardwareBase::write(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        if (!active_) {
            return hardware_interface::return_type::OK;
        }

        return PublishCommand() ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
    }

    bool UnitreeG1SimHardware::PublishCommand()
    {
        if (lowcmd_ == nullptr || lowstate_ == nullptr) {
            return false;
        }

        if (!lowcmd_->trylock()) {
            return true;
        }

        {
            std::lock_guard<std::mutex> state_lock(lowstate_->mutex_);
            lowcmd_->msg_.mode_pr() = lowstate_->msg_.mode_pr();
            lowcmd_->msg_.mode_machine() = lowstate_->msg_.mode_machine();
        }

        for (std::size_t i = 0; i < dds_indices_.size(); ++i)
        {
            auto& motor = lowcmd_->msg_.motor_cmd()[dds_indices_[i]];

            motor.mode() = 1;
            motor.q() = static_cast<float>(command_position_[i]);
            motor.dq() = 0.0F;
            motor.kp() = static_cast<float>(kp_[i]);
            motor.kd() = static_cast<float>(kd_[i]);
            motor.tau() = 0.0F;
        }

        lowcmd_->unlockAndPublish();

        return true;
    }

    UnitreeG1HardwareBase::CallbackReturn UnitreeG1ArmHardware::on_init(const hardware_interface::HardwareInfo& info)
    {
        const auto result = UnitreeG1HardwareBase::on_init(info);

        if (result != CallbackReturn::SUCCESS) {
            return result;
        }

        if (info_.hardware_parameters.count("arm_sdk_weight") > 0) {
            arm_sdk_weight_ = std::stod(info_.hardware_parameters.at("arm_sdk_weight"));
        }

        return CallbackReturn::SUCCESS;
    }

    bool UnitreeG1ArmHardware::ConfigureCommandPublisher()
    {
        arm_sdk_ = std::make_unique<unitree::robot::g1::publisher::ArmSdk>();
        return true;
    }

    bool UnitreeG1ArmHardware::PublishCommand()
    {
        if (arm_sdk_ == nullptr) {
            return false;
        }

        if (!arm_sdk_->trylock()) {
            return true;
        }

        arm_sdk_->weight(static_cast<float>(arm_sdk_weight_));

        for (std::size_t i = 0; i < dds_indices_.size(); ++i)
        {
            auto& motor = arm_sdk_->msg_.motor_cmd()[dds_indices_[i]];

            motor.mode() = 1;
            motor.q() = static_cast<float>(command_position_[i]);
            motor.dq() = 0.0F;
            motor.kp() = static_cast<float>(kp_[i]);
            motor.kd() = static_cast<float>(kd_[i]);
            motor.tau() = 0.0F;
        }

        arm_sdk_->unlockAndPublish();

        return true;
    }

    UnitreeG1HardwareBase::CallbackReturn UnitreeG1HardwareBase::on_deactivate(const rclcpp_lifecycle::State& previous_state)
    {
        active_ = false;
        return CallbackReturn::SUCCESS;
    }

    UnitreeG1HardwareBase::CallbackReturn UnitreeG1HardwareBase::on_cleanup(const rclcpp_lifecycle::State& previous_state)
    {
        active_ = false;
        lowstate_.reset();
        ResetCommandPublisher();

        return CallbackReturn::SUCCESS;
    }

    void UnitreeG1SimHardware::ResetCommandPublisher()
    {
        lowcmd_.reset();
    }

    void UnitreeG1ArmHardware::ResetCommandPublisher()
    {
        arm_sdk_.reset();
    }
}

PLUGINLIB_EXPORT_CLASS(xchallenge::g1_hardware::UnitreeG1SimHardware, hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(xchallenge::g1_hardware::UnitreeG1ArmHardware, hardware_interface::SystemInterface)
