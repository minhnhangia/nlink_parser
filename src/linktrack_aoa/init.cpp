#include "init.h"

#include <cstring>
#include <string>

#include "../linktrack/protocols.h"
#include "nlink_protocol.h"
#include "nlink_unpack/nlink_linktrack_aoa_nodeframe0.h"
#include "nlink_unpack/nlink_linktrack_nodeframe0.h"
#include "nutils.h"

class NLTAoa_ProtocolNodeFrame0 : public NLinkProtocolVLength {
public:
  NLTAoa_ProtocolNodeFrame0();

protected:
  void UnpackFrameData(const uint8_t *data) override;
};

NLTAoa_ProtocolNodeFrame0::NLTAoa_ProtocolNodeFrame0()
    : NLinkProtocolVLength(true, g_nltaoa_nodeframe0.fixed_part_size,
                           {g_nltaoa_nodeframe0.frame_header,
                            g_nltaoa_nodeframe0.function_mark}) {}

void NLTAoa_ProtocolNodeFrame0::UnpackFrameData(const uint8_t *data) {
  g_nltaoa_nodeframe0.UnpackData(data, length());
}

namespace linktrack_aoa {

nlink_parser::msg::LinktrackNodeframe0 g_msg_nodeframe0;
nlink_parser::msg::LinktrackAoaNodeframe0 g_msg_aoa_nodeframe0;

Init::Init(NProtocolExtracter *protocol_extraction, serial::Serial *serial,
           const rclcpp::Node::SharedPtr &node)
    : node_(node), serial_(serial) {
  initDataTransmission();
  initNodeFrame0(protocol_extraction);
  InitAoaNodeFrame0(protocol_extraction);
}

void Init::initDataTransmission() {
  if (!node_) {
    return;
  }
  dt_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "nlink_linktrack_data_transmission", rclcpp::QoS(1000),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (serial_) {
          serial_->write(msg->data);
        }
      });
}

void Init::initNodeFrame0(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLT_ProtocolNodeFrame0;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([this] {
    if (node_ && !nodeframe0_pub_) {
      const auto topic = std::string("nlink_linktrack_nodeframe0");
      nodeframe0_pub_ = node_->create_publisher<nlink_parser::msg::LinktrackNodeframe0>(
          topic, rclcpp::QoS(200));
      TopicAdvertisedTip(node_->get_logger(), topic);
    }
    const auto &data = g_nlt_nodeframe0.result;
    auto &msg_data = g_msg_nodeframe0;
    auto &msg_nodes = msg_data.nodes;

    msg_data.role = data.role;
    msg_data.id = data.id;

    msg_nodes.resize(data.valid_node_count);
    for (size_t i = 0; i < data.valid_node_count; ++i) {
      auto &msg_node = msg_nodes[i];
      auto node = data.nodes[i];
      msg_node.id = node->id;
      msg_node.role = node->role;
      msg_node.data.resize(node->data_length);
      std::memcpy(msg_node.data.data(), node->data, node->data_length);
    }

    if (nodeframe0_pub_) {
      nodeframe0_pub_->publish(msg_data);
    }
  });
}

void Init::InitAoaNodeFrame0(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLTAoa_ProtocolNodeFrame0;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([this] {
    if (node_ && !aoa_nodeframe0_pub_) {
      const auto topic = std::string("nlink_linktrack_aoa_nodeframe0");
      aoa_nodeframe0_pub_ =
          node_->create_publisher<nlink_parser::msg::LinktrackAoaNodeframe0>(
              topic, rclcpp::QoS(200));
      TopicAdvertisedTip(node_->get_logger(), topic);
    }
    const auto &data = g_nltaoa_nodeframe0.result;
    auto &msg_data = g_msg_aoa_nodeframe0;
    auto &msg_nodes = msg_data.nodes;

    msg_data.role = data.role;
    msg_data.id = data.id;
    msg_data.local_time = data.local_time;
    msg_data.system_time = data.system_time;
    msg_data.voltage = data.voltage;

    msg_nodes.resize(data.valid_node_count);
    for (size_t i = 0; i < data.valid_node_count; ++i) {
      auto &msg_node = msg_nodes[i];
      auto node = data.nodes[i];
      msg_node.id = node->id;
      msg_node.role = node->role;
      msg_node.dis = node->dis;
      msg_node.angle = node->angle;
      msg_node.fp_rssi = node->fp_rssi;
      msg_node.rx_rssi = node->rx_rssi;
    }

    if (aoa_nodeframe0_pub_) {
      aoa_nodeframe0_pub_->publish(msg_data);
    }
  });
}

} // namespace linktrack_aoa
