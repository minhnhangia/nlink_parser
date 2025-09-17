#ifndef TOFSENSEMINIT_H
#define TOFSENSEMINIT_H

#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "nlink_parser/msg/tofsense_m_frame0.hpp"
#include "protocol_extracter/nprotocol_extracter.h"

namespace tofsensem {
class Init {
public:
  explicit Init(NProtocolExtracter *protocol_extraction,
                const rclcpp::Node::SharedPtr &node = nullptr);

private:
  void InitFrame0(NProtocolExtracter *protocol_extraction);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nlink_parser::msg::TofsenseMFrame0>::SharedPtr frame0_pub_;
};

} // namespace tofsensem
#endif // TOFSENSEMINIT_H
