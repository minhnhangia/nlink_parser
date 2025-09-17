#ifndef LINKTRACKAOAINIT_H
#define LINKTRACKAOAINIT_H

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

#include <memory>

#include "nlink_parser/msg/linktrack_aoa_nodeframe0.hpp"
#include "nlink_parser/msg/linktrack_nodeframe0.hpp"
#include "protocol_extracter/nprotocol_extracter.h"
#include "std_msgs/msg/string.hpp"

namespace linktrack_aoa {
class Init {
public:
  explicit Init(NProtocolExtracter *protocol_extraction, serial::Serial *serial,
                const rclcpp::Node::SharedPtr &node = nullptr);

private:
  void initDataTransmission();
  void initNodeFrame0(NProtocolExtracter *protocol_extraction);
  void InitAoaNodeFrame0(NProtocolExtracter *protocol_extraction);

  rclcpp::Node::SharedPtr node_;
  serial::Serial *serial_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dt_sub_;
  rclcpp::Publisher<nlink_parser::msg::LinktrackNodeframe0>::SharedPtr
      nodeframe0_pub_;
  rclcpp::Publisher<nlink_parser::msg::LinktrackAoaNodeframe0>::SharedPtr
      aoa_nodeframe0_pub_;
};
} // namespace linktrack_aoa

#endif // LINKTRACKAOAINIT_H
