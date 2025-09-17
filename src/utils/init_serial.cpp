#include "init_serial.h"

#include <rclcpp/logging.hpp>

#include <string>
/*
void enumerate_ports() {
  auto devices_found = serial::list_ports();
  auto iter = devices_found.begin();
  while (iter != devices_found.end()) {
    serial::PortInfo device = *iter++;

    printf("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
           device.hardware_id.c_str());
  }
  std::string test;
  test.clear();
}
*/

void initSerial(serial::Serial *serial, const rclcpp::Node::SharedPtr &node) {
  try {
    auto logger = node ? node->get_logger() : rclcpp::get_logger("nlink_parser.serial");
    std::string port_name = "/dev/ttyUSB0";
    int baud_rate = 921600;
    if (node) {
      port_name = node->declare_parameter<std::string>("port_name", port_name);
      baud_rate = node->declare_parameter<int>("baud_rate", baud_rate);
    }

    serial->setPort(port_name);
    serial->setBaudrate(static_cast<uint32_t>(baud_rate));
    RCLCPP_INFO(logger, "try to open serial port with %s,%d", port_name.c_str(),
                baud_rate);
    auto timeout = serial::Timeout::simpleTimeout(10);
    // without setTimeout,serial can not write any data
    // https://stackoverflow.com/questions/52048670/can-read-but-cannot-write-serial-ports-on-ubuntu-16-04/52051660?noredirect=1#comment91056825_52051660
    serial->setTimeout(timeout);
    serial->open();

    if (serial->isOpen()) {
      RCLCPP_INFO(logger, "Serial port opened successfully, waiting for data.");
    } else {
      RCLCPP_ERROR(logger, "Failed to open serial port, please check and retry.");
      exit(EXIT_FAILURE);
    }
  } catch (const std::exception &e) {
    auto logger = node ? node->get_logger() : rclcpp::get_logger("nlink_parser.serial");
    RCLCPP_ERROR(logger, "Unhandled Exception: %s", e.what());
    exit(EXIT_FAILURE);
  }
}
