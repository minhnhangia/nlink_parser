#ifndef IOT_INIT_H
#define IOT_INIT_H

#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "nlink_parser/msg/iot_frame0.hpp"
#include "protocol_extracter/nprotocol_extracter.h"

namespace iot {
class Init {
public:
  explicit Init(NProtocolExtracter *protocol_extraction,
                const rclcpp::Node::SharedPtr &node = nullptr);

private:
  void InitFrame0(NProtocolExtracter *protocol_extraction);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nlink_parser::msg::IotFrame0>::SharedPtr frame0_pub_;
};

} // namespace iot
#endif // IOT_INIT_H
