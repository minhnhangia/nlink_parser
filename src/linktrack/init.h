#ifndef LINKTRACKINIT_H
#define LINKTRACKINIT_H

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

#include <memory>

#include "nlink_parser/msg/linktrack_anchorframe0.hpp"
#include "nlink_parser/msg/linktrack_nodeframe0.hpp"
#include "nlink_parser/msg/linktrack_nodeframe1.hpp"
#include "nlink_parser/msg/linktrack_nodeframe2.hpp"
#include "nlink_parser/msg/linktrack_nodeframe3.hpp"
#include "nlink_parser/msg/linktrack_nodeframe4.hpp"
#include "nlink_parser/msg/linktrack_nodeframe5.hpp"
#include "nlink_parser/msg/linktrack_nodeframe6.hpp"
#include "nlink_parser/msg/linktrack_tagframe0.hpp"
#include "protocol_extracter/nprotocol_extracter.h"
#include "std_msgs/msg/string.hpp"

namespace linktrack {
class Init {
public:
  explicit Init(NProtocolExtracter *protocol_extraction, serial::Serial *serial,
                const rclcpp::Node::SharedPtr &node = nullptr);

private:
  void initDataTransmission();
  void initAnchorFrame0(NProtocolExtracter *protocol_extraction);
  void initTagFrame0(NProtocolExtracter *protocol_extraction);
  void initNodeFrame0(NProtocolExtracter *protocol_extraction);
  void initNodeFrame1(NProtocolExtracter *protocol_extraction);
  void initNodeFrame2(NProtocolExtracter *protocol_extraction);
  void initNodeFrame3(NProtocolExtracter *protocol_extraction);
  void initNodeFrame4(NProtocolExtracter *protocol_extraction);
  void initNodeFrame5(NProtocolExtracter *protocol_extraction);
  void initNodeFrame6(NProtocolExtracter *protocol_extraction);

  rclcpp::Node::SharedPtr node_;
  serial::Serial *serial_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dt_sub_;

  rclcpp::Publisher<nlink_parser::msg::LinktrackAnchorframe0>::SharedPtr
      anchorframe0_pub_;
  rclcpp::Publisher<nlink_parser::msg::LinktrackTagframe0>::SharedPtr
      tagframe0_pub_;
  rclcpp::Publisher<nlink_parser::msg::LinktrackNodeframe0>::SharedPtr
      nodeframe0_pub_;
  rclcpp::Publisher<nlink_parser::msg::LinktrackNodeframe1>::SharedPtr
      nodeframe1_pub_;
  rclcpp::Publisher<nlink_parser::msg::LinktrackNodeframe2>::SharedPtr
      nodeframe2_pub_;
  rclcpp::Publisher<nlink_parser::msg::LinktrackNodeframe3>::SharedPtr
      nodeframe3_pub_;
  rclcpp::Publisher<nlink_parser::msg::LinktrackNodeframe4>::SharedPtr
      nodeframe4_pub_;
  rclcpp::Publisher<nlink_parser::msg::LinktrackNodeframe5>::SharedPtr
      nodeframe5_pub_;
  rclcpp::Publisher<nlink_parser::msg::LinktrackNodeframe6>::SharedPtr
      nodeframe6_pub_;
};
} // namespace linktrack

#endif // LINKTRACKINIT_H
