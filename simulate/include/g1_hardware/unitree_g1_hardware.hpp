#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <unitree/dds_wrapper/robots/g1/g1.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace xchallenge::g1_hardware
{

    class UnitreeG1HardwareBase : public hardware_interface::SystemInterface
    {
    public:
        using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

        CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    protected:
        virtual bool ValidateJointSet() const = 0;
        virtual bool ConfigureCommandPublisher() = 0;
        virtual bool PublishCommand() = 0;
        virtual void ResetCommandPublisher() = 0;

        std::size_t DdsIndexForJoint(const std::string& joint_name) const;

        int domain_id_ = 1;
        std::string network_interface_ = "lo";

        double default_kp_ = 40.0;
        double default_kd_ = 1.0;

        std::vector<std::size_t> dds_indices_;

        std::vector<double> state_position_;
        std::vector<double> state_velocity_;
        std::vector<double> state_effort_;

        std::vector<double> command_position_;

        std::vector<double> kp_;
        std::vector<double> kd_;

        std::shared_ptr<unitree::robot::g1::subscription::LowState> lowstate_;

        bool active_ = false;
    };


    class UnitreeG1SimHardware final : public UnitreeG1HardwareBase
    {
    protected:
        bool ValidateJointSet() const override;
        bool ConfigureCommandPublisher() override;
        bool PublishCommand() override;
        void ResetCommandPublisher() override;

    private:
        std::unique_ptr<unitree::robot::g1::publisher::LowCmd> lowcmd_;
    };


    class UnitreeG1ArmHardware final : public UnitreeG1HardwareBase
    {
    public:
        CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    protected:
        bool ValidateJointSet() const override;
        bool ConfigureCommandPublisher() override;
        bool PublishCommand() override;
        void ResetCommandPublisher() override;

    private:
        double arm_sdk_weight_ = 1.0;
        std::unique_ptr<unitree::robot::g1::publisher::ArmSdk> arm_sdk_;
    };

}