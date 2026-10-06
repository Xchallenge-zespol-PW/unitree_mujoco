#include <chrono>
#include <iostream>
#include <thread>

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

int main()
{
    unitree::robot::ChannelFactory::Instance()->Init(1, "eth2");

    auto subscriber = std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>("rt/lowstate");

    subscriber->InitChannel([](const void* message)
    {
        const auto* state = static_cast<const unitree_hg::msg::dds_::LowState_*>(message);

        std::cout << "LowState received | tick=" << state->tick() << " | q0=" << state->motor_state()[0].q() << std::endl;
    }, 1);

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}