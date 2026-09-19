#include <iostream>
#include <string>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>

#include "g1_loco_mode_options.hpp"

namespace {

void print_usage(const char* program) {
    std::cout << "Usage: " << program << " networkInterface {status|stop|damp|zero}" << std::endl
              << "  status : read G1 loco FSM/balance state only" << std::endl
              << "  stop   : request official loco StopMove()" << std::endl
              << "  damp   : request official loco Damp() / SetFsmId(1)" << std::endl
              << "  zero   : request official loco ZeroTorque() / SetFsmId(0)" << std::endl
              << "  services : read robot_state service list only" << std::endl
              << "  service-off SERVICE : request robot_state ServiceSwitch(SERVICE, 0)" << std::endl
              << "  service-on SERVICE  : request robot_state ServiceSwitch(SERVICE, 1)" << std::endl;
}

bool check_call(const std::string& name, int32_t ret) {
    std::cout << name << " ret=" << ret << std::endl;
    return ret == 0;
}

bool print_status(unitree::robot::g1::LocoClient& client) {
    int fsm_id = -1;
    int fsm_mode = -1;
    int balance_mode = -1;

    bool ok = true;
    ok = check_call("GetFsmId", client.GetFsmId(fsm_id)) && ok;
    ok = check_call("GetFsmMode", client.GetFsmMode(fsm_mode)) && ok;
    ok = check_call("GetBalanceMode", client.GetBalanceMode(balance_mode)) && ok;

    std::cout << "status fsm_id=" << fsm_id
              << " fsm_mode=" << fsm_mode
              << " balance_mode=" << balance_mode
              << std::endl;
    return ok;
}

bool print_services() {
    unitree::robot::go2::RobotStateClient client;
    client.SetTimeout(3.0f);
    client.Init();

    std::vector<unitree::robot::go2::ServiceState> services;
    int32_t ret = client.ServiceList(services);
    if (!check_call("ServiceList", ret)) {
        return false;
    }

    std::cout << "services count=" << services.size() << std::endl;
    for (const auto& service : services) {
        std::cout << "service name=" << service.name
                  << " status=" << service.status
                  << " protect=" << service.protect
                  << std::endl;
    }
    return true;
}

bool switch_service(const std::string& service_name, int32_t swit) {
    unitree::robot::go2::RobotStateClient client;
    client.SetTimeout(3.0f);
    client.Init();

    int32_t status = -1;
    int32_t ret = client.ServiceSwitch(service_name, swit, status);
    std::cout << "ServiceSwitch name=" << service_name
              << " switch=" << swit
              << " ret=" << ret
              << " returned_status=" << status
              << std::endl;
    return ret == 0;
}

}  // namespace

int main(int argc, char const* argv[]) {
    G1LocoModeOptions options = parse_g1_loco_mode_options(argc, argv);
    if (!options.valid) {
        print_usage(argv[0]);
        return 2;
    }

    unitree::robot::ChannelFactory::Instance()->Init(0, options.network_interface);

    if (options.command == G1LocoModeCommand::Services) {
        std::cout << "Connected to robot_state service on " << options.network_interface << std::endl;
        return print_services() ? 0 : 1;
    }
    if (options.command == G1LocoModeCommand::ServiceOff || options.command == G1LocoModeCommand::ServiceOn) {
        std::cout << "Connected to robot_state service on " << options.network_interface << std::endl;
        const int32_t swit = options.command == G1LocoModeCommand::ServiceOn ? 1 : 0;
        bool ok = switch_service(options.service_name, swit);
        ok = print_services() && ok;
        return ok ? 0 : 1;
    }

    unitree::robot::g1::LocoClient client;
    client.SetTimeout(3.0f);
    client.Init();

    std::cout << "Connected to G1 loco service on " << options.network_interface << std::endl;

    if (options.command == G1LocoModeCommand::Status) {
        return print_status(client) ? 0 : 1;
    }

    std::cout << "Initial ";
    bool ok = print_status(client);

    if (options.command == G1LocoModeCommand::Stop) {
        ok = check_call("StopMove", client.StopMove()) && ok;
    } else if (options.command == G1LocoModeCommand::Damp) {
        std::cout << "Requesting official Damp(): robot may soften. Keep it supported." << std::endl;
        ok = check_call("Damp", client.Damp()) && ok;
    } else if (options.command == G1LocoModeCommand::ZeroTorque) {
        std::cout << "Requesting official ZeroTorque(): robot may lose support immediately." << std::endl;
        ok = check_call("ZeroTorque", client.ZeroTorque()) && ok;
    }

    std::cout << "After command ";
    ok = print_status(client) && ok;

    return ok ? 0 : 1;
}
