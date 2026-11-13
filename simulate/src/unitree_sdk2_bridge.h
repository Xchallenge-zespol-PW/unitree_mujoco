#pragma once

#include <mujoco/mujoco.h>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/robots/go2/go2.h>
#include <unitree/dds_wrapper/robots/g1/g1.h>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>

#include <atomic>
#include <iostream>

#include "param.h"
#include "physics_joystick.h"
#include <type_traits>

#define MOTOR_SENSOR_NUM 3

// Set once a controller sends a lowcmd with any non-zero kp, kd or tau; cleared on reset.
inline std::atomic<bool> lowcmd_active{false};

class UnitreeSDK2BridgeBase;
// The bridge the physics thread drives (set by UnitreeSdk2BridgeThread).
inline std::atomic<UnitreeSDK2BridgeBase*> active_bridge{nullptr};

class UnitreeSDK2BridgeBase
{
public:
    // Called by the physics thread with the simulation mutex held.
    // Joint torques of the motors' PD law from the latest lowcmd and the current state, every step.
    virtual void ApplyCommand() {}
    // Publishes rt/lowstate from the current state (lockstep: after every step).
    virtual void PublishState() {}
    // Tick (lowstate) of the state the latest lowcmd was computed from.
    virtual uint32_t CommandTick() { return 0; }
    // Tick of the state published last (lockstep), and that state sent again unchanged.
    virtual uint32_t StateTick() { return 0; }
    virtual void RepublishState() {}

    UnitreeSDK2BridgeBase(mjModel *model, mjData *data)
    : mj_model_(model), mj_data_(data)
    {
        _check_sensor();
        if(param::config.print_scene_information == 1) {
            printSceneInformation();
        }
        if(param::config.use_joystick == 1) {
            if(param::config.joystick_type == "xbox") {
                joystick = std::make_shared<XBoxJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "switch") {
                joystick  = std::make_shared<SwitchJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else {
                std::cerr << "Unsupported joystick type: " << param::config.joystick_type << std::endl;
                exit(EXIT_FAILURE);
            }
        }

    }
    virtual ~UnitreeSDK2BridgeBase() = default;
    virtual void start() {}
    virtual void stop() {}

    void printSceneInformation()
    {
        auto printObjects = [this](const char* title, int count, int type, auto getIndex) {
            std::cout << "<<------------- " << title << " ------------->> " << std::endl;
            for (int i = 0; i < count; i++) {
                const char* name = mj_id2name(mj_model_, type, i);
                if (name) {
                    std::cout << title << "_index: " << getIndex(i) << ", " << "name: " << name;
                    if (type == mjOBJ_SENSOR) {
                        std::cout << ", dim: " << mj_model_->sensor_dim[i];
                    }
                    std::cout << std::endl;
                }
            }
            std::cout << std::endl;
        };
    
        printObjects("Link", mj_model_->nbody, mjOBJ_BODY, [](int i) { return i; });
        printObjects("Joint", mj_model_->njnt, mjOBJ_JOINT, [](int i) { return i; });
        printObjects("Actuator", mj_model_->nu, mjOBJ_ACTUATOR, [](int i) { return i; });
    
        int sensorIndex = 0;
        printObjects("Sensor", mj_model_->nsensor, mjOBJ_SENSOR, [&](int i) {
            int currentIndex = sensorIndex;
            sensorIndex += mj_model_->sensor_dim[i];
            return currentIndex;
        });
    }

protected:
    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    mjData *mj_data_;
    mjModel *mj_model_;

    // Sensor data indices
    int imu_quat_adr_ = -1;
    int imu_gyro_adr_ = -1;
    int imu_acc_adr_ = -1;
    int frame_pos_adr_ = -1;
    int frame_vel_adr_ = -1;

    int secondary_imu_quat_adr_ = -1;
    int secondary_imu_gyro_adr_ = -1;
    int secondary_imu_acc_adr_ = -1;

    std::shared_ptr<unitree::common::UnitreeJoystick> joystick = nullptr;

    void _check_sensor()
    {
        num_motor_ = mj_model_->nu;
        dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;
    
        // Find sensor addresses by name
        int sensor_id = -1;
        
        // IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_quat");
        if (sensor_id >= 0) {
            imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_gyro");
        if (sensor_id >= 0) {
            imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_acc");
        if (sensor_id >= 0) {
            imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame position
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_pos");
        if (sensor_id >= 0) {
            frame_pos_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame velocity
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_vel");
        if (sensor_id >= 0) {
            frame_vel_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_quat");
        if (sensor_id >= 0) {
            secondary_imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_gyro");
        if (sensor_id >= 0) {
            secondary_imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_acc");
        if (sensor_id >= 0) {
            secondary_imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }
    }
};

template <typename LowCmd_t, typename LowState_t>
class RobotBridge : public UnitreeSDK2BridgeBase
{
using HighState_t = unitree::robot::go2::publisher::SportModeState;
using WirelessController_t = unitree::robot::go2::publisher::WirelessController;

public:
    RobotBridge(mjModel *model, mjData *data) : UnitreeSDK2BridgeBase(model, data)
    {
        lowcmd = std::make_shared<LowCmd_t>("rt/lowcmd");
        lowstate = std::make_unique<LowState_t>();
        lowstate->joystick = joystick;
        highstate = std::make_unique<HighState_t>();
        wireless_controller = std::make_unique<WirelessController_t>();
        wireless_controller->joystick = joystick;
    }

    ~RobotBridge() override
    {
        stop();
    }

    void start() override
    {
        thread_ = std::make_shared<unitree::common::RecurrentThread>("unitree_bridge", UT_CPU_ID_NONE, 1000, [this]() { this->run(); });
    }

    void stop() override
    {
        thread_.reset();
    }

    void ApplyCommand() override
    {
        if(!mj_data_) return;
        std::lock_guard<std::mutex> lock(lowcmd->mutex_);
        bool active = false;
        for(int i(0); i<num_motor_; i++) {
            auto & m = lowcmd->msg_.motor_cmd()[i];
            mj_data_->ctrl[i] = m.tau() +
                                m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
            active = active || m.kp() != 0 || m.kd() != 0 || m.tau() != 0;
        }
        if(active) { lowcmd_active.store(true); }
    }

    uint32_t CommandTick() override
    {
        if constexpr (std::is_same_v<LowCmd_t, unitree::robot::g1::subscription::LowCmd>) {
            std::lock_guard<std::mutex> lock(lowcmd->mutex_);
            return lowcmd->msg_.reserve()[0];
        }
        return 0;
    }

    void PublishState() override
    {
        if(!mj_data_) return;
        if(param::config.lockstep == 1) {
            // Written directly: RealTimePublisher sends at most once per millisecond from its thread and
            // drops a state while the last one is unsent, and the controller must see every step.
            if(!lowstate_direct_) {
                lowstate_direct_ = std::make_shared<unitree::robot::ChannelPublisher<typename LowState_t::MsgType>>("rt/lowstate");
                lowstate_direct_->InitChannel();
                lowstate->lock();
                state_msg_ = lowstate->msg_;
                lowstate->unlock();
            }
            FillState(state_msg_);
            lowstate_direct_->Write(state_msg_, 0);
            state_tick_ = state_msg_.tick();
            return;
        }
        if(lowstate->trylock()) {
            FillState(lowstate->msg_);
            lowstate->unlockAndPublish();
        }
    }

    uint32_t StateTick() override { return state_tick_; }

    void RepublishState() override
    {
        if(lowstate_direct_) { lowstate_direct_->Write(state_msg_, 0); }
    }

    template <typename Msg>
    void FillState(Msg & msg)
    {
        {
            for(int i(0); i<num_motor_; i++) {
                msg.motor_state()[i].q() = mj_data_->sensordata[i];
                msg.motor_state()[i].dq() = mj_data_->sensordata[i + num_motor_];
                msg.motor_state()[i].tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
            }
            
            if(imu_quat_adr_ >= 0) {
                msg.imu_state().quaternion()[0] = mj_data_->sensordata[imu_quat_adr_ + 0];
                msg.imu_state().quaternion()[1] = mj_data_->sensordata[imu_quat_adr_ + 1];
                msg.imu_state().quaternion()[2] = mj_data_->sensordata[imu_quat_adr_ + 2];
                msg.imu_state().quaternion()[3] = mj_data_->sensordata[imu_quat_adr_ + 3];

                double w = msg.imu_state().quaternion()[0];
                double x = msg.imu_state().quaternion()[1];
                double y = msg.imu_state().quaternion()[2];
                double z = msg.imu_state().quaternion()[3];

                msg.imu_state().rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                msg.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
                msg.imu_state().rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }
            
            if(imu_gyro_adr_ >= 0) {
                msg.imu_state().gyroscope()[0] = mj_data_->sensordata[imu_gyro_adr_ + 0];
                msg.imu_state().gyroscope()[1] = mj_data_->sensordata[imu_gyro_adr_ + 1];
                msg.imu_state().gyroscope()[2] = mj_data_->sensordata[imu_gyro_adr_ + 2];
            }

            if(imu_acc_adr_ >= 0) {
                msg.imu_state().accelerometer()[0] = mj_data_->sensordata[imu_acc_adr_ + 0];
                msg.imu_state().accelerometer()[1] = mj_data_->sensordata[imu_acc_adr_ + 1];
                msg.imu_state().accelerometer()[2] = mj_data_->sensordata[imu_acc_adr_ + 2];
            }
            
            msg.tick() = std::round(mj_data_->time / 1e-3);
        }
    }

    virtual void run()
    {
        if(!mj_data_) return;
        if(lowstate->joystick) { lowstate->joystick->update(); }
        // The PD law runs in the physics step (ApplyCommand); in lockstep the physics thread also
        // publishes the low state, right after each step.
        if(param::config.lockstep != 1) { PublishState(); }
        // highstate
        if(highstate->trylock()) {
            if(frame_pos_adr_ >= 0) {
                highstate->msg_.position()[0] = mj_data_->sensordata[frame_pos_adr_ + 0];
                highstate->msg_.position()[1] = mj_data_->sensordata[frame_pos_adr_ + 1];
                highstate->msg_.position()[2] = mj_data_->sensordata[frame_pos_adr_ + 2];
            }
            if(frame_vel_adr_ >= 0) {
                highstate->msg_.velocity()[0] = mj_data_->sensordata[frame_vel_adr_ + 0];
                highstate->msg_.velocity()[1] = mj_data_->sensordata[frame_vel_adr_ + 1];
                highstate->msg_.velocity()[2] = mj_data_->sensordata[frame_vel_adr_ + 2];
            }
            highstate->unlockAndPublish();
        }
        // wireless_controller
        if(wireless_controller->joystick) {
            wireless_controller->unlockAndPublish();
        }
    }

    std::unique_ptr<HighState_t> highstate;
    std::unique_ptr<WirelessController_t> wireless_controller;
    std::shared_ptr<LowCmd_t> lowcmd;
    std::unique_ptr<LowState_t> lowstate;
    std::shared_ptr<unitree::robot::ChannelPublisher<typename LowState_t::MsgType>> lowstate_direct_;
    typename LowState_t::MsgType state_msg_{};
    std::atomic<uint32_t> state_tick_{0};
    
private:
    unitree::common::RecurrentThreadPtr thread_;
};

using Go2Bridge = RobotBridge<unitree::robot::go2::subscription::LowCmd, unitree::robot::go2::publisher::LowState>;

class G1Bridge : public RobotBridge<unitree::robot::g1::subscription::LowCmd, unitree::robot::g1::publisher::LowState>
{
public:
    G1Bridge(mjModel *model, mjData *data) : RobotBridge(model, data)
    {
        if (param::config.robot.find("g1") != std::string::npos) {
            auto* g1_lowstate = dynamic_cast<unitree::robot::g1::publisher::LowState*>(lowstate.get());
            if (g1_lowstate) {
                auto scene = param::config.robot_scene.filename().string();
                g1_lowstate->msg_.mode_machine() = scene.find("23") != std::string::npos ? 4 : 5;
            }
        }

        bmsstate = std::make_unique<BmsState_t>("rt/lf/bmsstate");
        bmsstate->msg_.soc() = 100;

        secondary_imustate = std::make_unique<IMUState_t>("rt/secondary_imu");
    }

    ~G1Bridge() override
    {
        stop();
    }

    void run() override
    {
        RobotBridge::run();

        // secondary IMU state
        if (secondary_imustate->trylock()) {
            if(secondary_imu_quat_adr_ >= 0) {
                secondary_imustate->msg_.quaternion()[0] = mj_data_->sensordata[secondary_imu_quat_adr_ + 0];
                secondary_imustate->msg_.quaternion()[1] = mj_data_->sensordata[secondary_imu_quat_adr_ + 1];
                secondary_imustate->msg_.quaternion()[2] = mj_data_->sensordata[secondary_imu_quat_adr_ + 2];
                secondary_imustate->msg_.quaternion()[3] = mj_data_->sensordata[secondary_imu_quat_adr_ + 3];

                double w = secondary_imustate->msg_.quaternion()[0];
                double x = secondary_imustate->msg_.quaternion()[1];
                double y = secondary_imustate->msg_.quaternion()[2];
                double z = secondary_imustate->msg_.quaternion()[3];

                secondary_imustate->msg_.rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                secondary_imustate->msg_.rpy()[1] = asin(2 * (w * y - z * x));
                secondary_imustate->msg_.rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }

            if(secondary_imu_gyro_adr_ >= 0) {
                secondary_imustate->msg_.gyroscope()[0] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 0];
                secondary_imustate->msg_.gyroscope()[1] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 1];
                secondary_imustate->msg_.gyroscope()[2] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 2];
            }

            if(secondary_imu_acc_adr_ >= 0) {
                secondary_imustate->msg_.accelerometer()[0] = mj_data_->sensordata[secondary_imu_acc_adr_ + 0];
                secondary_imustate->msg_.accelerometer()[1] = mj_data_->sensordata[secondary_imu_acc_adr_ + 1];
                secondary_imustate->msg_.accelerometer()[2] = mj_data_->sensordata[secondary_imu_acc_adr_ + 2];
            }

            secondary_imustate->unlockAndPublish();
        }

        // In practice, bmsstate is sent at a low frequency; here it is sent with the main loop
        bmsstate->unlockAndPublish();
    }

    using BmsState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::BmsState_>;
    using IMUState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::IMUState_>;
    std::unique_ptr<BmsState_t> bmsstate;
    std::unique_ptr<IMUState_t> secondary_imustate;
};
