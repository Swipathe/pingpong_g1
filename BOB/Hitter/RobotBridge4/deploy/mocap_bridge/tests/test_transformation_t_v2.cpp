#include "unitree_sdk2/lcm_types/transformation_t.hpp"

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
  lcm_types::transformation_t msg{};
  msg.name = "ball";
  msg.vicon_frame_number = 41;
  msg.vicon_time_s = 0.125;
  msg.publish_time_us = 1700000000000000LL;
  msg.track_id = 9001;
  msg.valid = 1;
  msg.occluded = 0;
  msg.pos_vicon[0] = 0.7;
  msg.pos_vicon[1] = -0.2;
  msg.pos_vicon[2] = 1.0;
  msg.quat_vicon[3] = 1.0;

  std::vector<unsigned char> wire(msg.getEncodedSize());
  if (msg.encode(wire.data(), 0, wire.size()) < 0) return 2;
  if (argc == 2 && std::string(argv[1]) == "--emit-hex") {
    for (unsigned char byte : wire) {
      std::cout << std::hex << std::setw(2) << std::setfill('0') << int(byte);
    }
    std::cout << "\n";
  }
  return msg.track_id == 9001 ? 0 : 3;
}
