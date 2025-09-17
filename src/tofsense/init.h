#ifndef TOFSENSEINIT_H
#define TOFSENSEINIT_H

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

#include <cstdint>
#include <map>
#include <memory>

#include "nlink_parser/msg/tofsense_cascade.hpp"
#include "nlink_parser/msg/tofsense_frame0.hpp"
#include "protocol_extracter/nprotocol_extracter.h"

namespace tofsense {
class Init {
public:
  explicit Init(NProtocolExtracter *protocol_extraction, serial::Serial *serial,
                const rclcpp::Node::SharedPtr &node = nullptr);

private:
  void InitFrame0(NProtocolExtracter *protocol_extraction);

  rclcpp::Node::SharedPtr node_;
  serial::Serial *serial_;

  rclcpp::Publisher<nlink_parser::msg::TofsenseCascade>::SharedPtr cascade_pub_;
  rclcpp::Publisher<nlink_parser::msg::TofsenseFrame0>::SharedPtr frame0_pub_;

  std::map<int, nlink_parser::msg::TofsenseFrame0> frame0_map_;

  const int frequency_ = 10;
  bool is_inquire_mode_ = true;

  rclcpp::TimerBase::SharedPtr timer_scan_;
  rclcpp::TimerBase::SharedPtr timer_read_;
  uint8_t node_index_ = 0;
};

} // namespace tofsense
#endif // TOFSENSEINIT_H
