#pragma once

#include <string>

enum class G1LocoModeCommand {
    Status,
    Stop,
    Damp,
    ZeroTorque,
    Services,
    ServiceOff,
    ServiceOn,
    Unknown,
};

struct G1LocoModeOptions {
    std::string network_interface;
    std::string service_name;
    G1LocoModeCommand command = G1LocoModeCommand::Unknown;
    bool valid = false;
};

inline G1LocoModeCommand parse_g1_loco_mode_command(const std::string& command) {
    if (command == "status") {
        return G1LocoModeCommand::Status;
    }
    if (command == "stop") {
        return G1LocoModeCommand::Stop;
    }
    if (command == "damp") {
        return G1LocoModeCommand::Damp;
    }
    if (command == "zero") {
        return G1LocoModeCommand::ZeroTorque;
    }
    if (command == "services") {
        return G1LocoModeCommand::Services;
    }
    if (command == "service-off") {
        return G1LocoModeCommand::ServiceOff;
    }
    if (command == "service-on") {
        return G1LocoModeCommand::ServiceOn;
    }
    return G1LocoModeCommand::Unknown;
}

inline G1LocoModeOptions parse_g1_loco_mode_options(int argc, char const* argv[]) {
    G1LocoModeOptions options;
    if (argc < 3 || argv[1] == nullptr || argv[2] == nullptr) {
        return options;
    }

    options.network_interface = argv[1];
    options.command = parse_g1_loco_mode_command(argv[2]);
    if (options.command == G1LocoModeCommand::ServiceOff || options.command == G1LocoModeCommand::ServiceOn) {
        if (argc < 4 || argv[3] == nullptr) {
            return options;
        }
        options.service_name = argv[3];
    }
    bool service_name_ok = true;
    if (options.command == G1LocoModeCommand::ServiceOff || options.command == G1LocoModeCommand::ServiceOn) {
        service_name_ok = !options.service_name.empty();
    }
    options.valid = !options.network_interface.empty()
        && options.command != G1LocoModeCommand::Unknown
        && service_name_ok;
    return options;
}
