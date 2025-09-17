#include <rclcpp/rclcpp.hpp>

#include "init.h"
#include "init_serial.h"
#include "protocol_extracter/nprotocol_extracter.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("iot_parser");
  serial::Serial serial;
  initSerial(&serial, node);
  NProtocolExtracter protocol_extraction;
  iot::Init init(&protocol_extraction, node);
  rclcpp::WallRate loop_rate(1000);
  while (rclcpp::ok()) {
    auto available_bytes = serial.available();
    std::string str_received;
    if (available_bytes) {
      serial.read(str_received, available_bytes);
      protocol_extraction.AddNewData(str_received);
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
