#include <array>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <thread>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowState_.hpp>

#include "assets/remote_controller.hpp"

static const std::string HG_STATE_TOPIC = "rt/lowstate";

class RemoteLowStateProbe {
public:
    explicit RemoteLowStateProbe(const std::string &network_interface) {
        unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
        lowstate_subscriber_.reset(
            new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(
                HG_STATE_TOPIC));
        lowstate_subscriber_->InitChannel(
            std::bind(&RemoteLowStateProbe::lowstateHandler, this, std::placeholders::_1), 1);
    }

private:
    void lowstateHandler(const void *message) {
        const auto *low_state = static_cast<const unitree_hg::msg::dds_::LowState_ *>(message);
        std::array<uint8_t, 40> raw{};
        for (size_t i = 0; i < raw.size(); ++i) {
            raw[i] = low_state->wireless_remote()[i];
        }

        xRockerBtnDataStruct remote{};
        std::memcpy(&remote, raw.data(), raw.size());

        const auto now = std::chrono::steady_clock::now();
        const bool changed = raw != last_raw_;
        const bool due = (now - last_print_) > std::chrono::milliseconds(500);
        if (!changed && !due) {
            return;
        }

        last_raw_ = raw;
        last_print_ = now;
        ++print_count_;

        std::cout << "sample=" << print_count_
                  << " mode_machine=" << static_cast<int>(low_state->mode_machine())
                  << " buttons"
                  << " A=" << static_cast<int>(remote.btn.components.A)
                  << " B=" << static_cast<int>(remote.btn.components.B)
                  << " X=" << static_cast<int>(remote.btn.components.X)
                  << " Y=" << static_cast<int>(remote.btn.components.Y)
                  << " L1=" << static_cast<int>(remote.btn.components.L1)
                  << " L2=" << static_cast<int>(remote.btn.components.L2)
                  << " R1=" << static_cast<int>(remote.btn.components.R1)
                  << " R2=" << static_cast<int>(remote.btn.components.R2)
                  << " lx=" << remote.lx
                  << " ly=" << remote.ly
                  << " rx=" << remote.rx
                  << " ry=" << remote.ry
                  << " raw=";
        for (const auto byte : raw) {
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int>(byte) << " ";
        }
        std::cout << std::dec << std::setfill(' ') << std::endl;
    }

    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_>
        lowstate_subscriber_;
    std::array<uint8_t, 40> last_raw_{};
    std::chrono::steady_clock::time_point last_print_{};
    uint64_t print_count_ = 0;
};

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <network-interface>" << std::endl;
        return 2;
    }

    RemoteLowStateProbe probe(argv[1]);
    std::cout << "Listening rt/lowstate remote bytes. Press remote buttons now." << std::endl;
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return 0;
}
