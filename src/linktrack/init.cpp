#include "init.h"

#include <cstring>
#include <string>

#include "nutils.h"
#include "protocols.h"

#define ARRAY_ASSIGN(DEST, SRC)                                                \
  for (size_t _CNT = 0; _CNT < sizeof(SRC) / sizeof(SRC[0]); ++_CNT) {         \
    DEST[_CNT] = SRC[_CNT];                                                    \
  }

namespace linktrack {

nlink_parser::msg::LinktrackAnchorframe0 g_msg_anchorframe0;
nlink_parser::msg::LinktrackTagframe0 g_msg_tagframe0;
nlink_parser::msg::LinktrackNodeframe0 g_msg_nodeframe0;
nlink_parser::msg::LinktrackNodeframe1 g_msg_nodeframe1;
nlink_parser::msg::LinktrackNodeframe2 g_msg_nodeframe2;
nlink_parser::msg::LinktrackNodeframe3 g_msg_nodeframe3;
nlink_parser::msg::LinktrackNodeframe4 g_msg_nodeframe4;
nlink_parser::msg::LinktrackNodeframe5 g_msg_nodeframe5;
nlink_parser::msg::LinktrackNodeframe6 g_msg_nodeframe6;

Init::Init(NProtocolExtracter *protocol_extraction, serial::Serial *serial,
           const rclcpp::Node::SharedPtr &node)
    : node_(node), serial_(serial) {
  initDataTransmission();
  initAnchorFrame0(protocol_extraction);
  initTagFrame0(protocol_extraction);
  initNodeFrame0(protocol_extraction);
  initNodeFrame1(protocol_extraction);
  initNodeFrame2(protocol_extraction);
  initNodeFrame3(protocol_extraction);
  initNodeFrame4(protocol_extraction);
  initNodeFrame5(protocol_extraction);
  initNodeFrame6(protocol_extraction);
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

void Init::initAnchorFrame0(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLT_ProtocolAnchorFrame0;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([this] {
    if (node_ && !anchorframe0_pub_) {
      const auto topic = std::string("nlink_linktrack_anchorframe0");
      anchorframe0_pub_ = node_->create_publisher<nlink_parser::msg::LinktrackAnchorframe0>(
          topic, rclcpp::QoS(200));
      TopicAdvertisedTip(node_->get_logger(), topic);
    }

    auto data = nlt_anchorframe0_.result;
    g_msg_anchorframe0.role = data.role;
    g_msg_anchorframe0.id = data.id;
    g_msg_anchorframe0.voltage = data.voltage;
    g_msg_anchorframe0.local_time = data.local_time;
    g_msg_anchorframe0.system_time = data.system_time;
    auto &msg_nodes = g_msg_anchorframe0.nodes;
    msg_nodes.clear();
    decltype(g_msg_anchorframe0.nodes)::value_type msg_node;
    for (size_t i = 0, icount = data.valid_node_count; i < icount; ++i) {
      auto node = data.nodes[i];
      msg_node.role = node->role;
      msg_node.id = node->id;
      ARRAY_ASSIGN(msg_node.pos_3d, node->pos_3d)
      ARRAY_ASSIGN(msg_node.dis_arr, node->dis_arr)
      msg_nodes.push_back(msg_node);
    }
    if (anchorframe0_pub_) {
      anchorframe0_pub_->publish(g_msg_anchorframe0);
    }
  });
}

void Init::initTagFrame0(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLT_ProtocolTagFrame0;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([this] {
    if (node_ && !tagframe0_pub_) {
      const auto topic = std::string("nlink_linktrack_tagframe0");
      tagframe0_pub_ = node_->create_publisher<nlink_parser::msg::LinktrackTagframe0>(
          topic, rclcpp::QoS(200));
      TopicAdvertisedTip(node_->get_logger(), topic);
    }

    const auto &data = g_nlt_tagframe0.result;
    auto &msg_data = g_msg_tagframe0;

    msg_data.role = data.role;
    msg_data.id = data.id;
    msg_data.local_time = data.local_time;
    msg_data.system_time = data.system_time;
    msg_data.voltage = data.voltage;
    ARRAY_ASSIGN(msg_data.pos_3d, data.pos_3d)
    ARRAY_ASSIGN(msg_data.eop_3d, data.eop_3d)
    ARRAY_ASSIGN(msg_data.vel_3d, data.vel_3d)
    ARRAY_ASSIGN(msg_data.dis_arr, data.dis_arr)
    ARRAY_ASSIGN(msg_data.imu_gyro_3d, data.imu_gyro_3d)
    ARRAY_ASSIGN(msg_data.imu_acc_3d, data.imu_acc_3d)
    ARRAY_ASSIGN(msg_data.angle_3d, data.angle_3d)
    ARRAY_ASSIGN(msg_data.quaternion, data.quaternion)

    if (tagframe0_pub_) {
      tagframe0_pub_->publish(msg_data);
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

void Init::initNodeFrame1(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLT_ProtocolNodeFrame1;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([this] {
    if (node_ && !nodeframe1_pub_) {
      const auto topic = std::string("nlink_linktrack_nodeframe1");
      nodeframe1_pub_ = node_->create_publisher<nlink_parser::msg::LinktrackNodeframe1>(
          topic, rclcpp::QoS(200));
      TopicAdvertisedTip(node_->get_logger(), topic);
    }
    const auto &data = g_nlt_nodeframe1.result;
    auto &msg_data = g_msg_nodeframe1;
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
      ARRAY_ASSIGN(msg_node.pos_3d, node->pos_3d)
    }

    if (nodeframe1_pub_) {
      nodeframe1_pub_->publish(msg_data);
    }
  });
}

void Init::initNodeFrame2(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLT_ProtocolNodeFrame2;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([this] {
    if (node_ && !nodeframe2_pub_) {
      const auto topic = std::string("nlink_linktrack_nodeframe2");
      nodeframe2_pub_ = node_->create_publisher<nlink_parser::msg::LinktrackNodeframe2>(
          topic, rclcpp::QoS(200));
      TopicAdvertisedTip(node_->get_logger(), topic);
    }
    const auto &data = g_nlt_nodeframe2.result;
    auto &msg_data = g_msg_nodeframe2;
    auto &msg_nodes = msg_data.nodes;

    msg_data.role = data.role;
    msg_data.id = data.id;
    msg_data.local_time = data.local_time;
    msg_data.system_time = data.system_time;
    msg_data.voltage = data.voltage;
    ARRAY_ASSIGN(msg_data.pos_3d, data.pos_3d)
    ARRAY_ASSIGN(msg_data.eop_3d, data.eop_3d)
    ARRAY_ASSIGN(msg_data.vel_3d, data.vel_3d)
    ARRAY_ASSIGN(msg_data.imu_gyro_3d, data.imu_gyro_3d)
    ARRAY_ASSIGN(msg_data.imu_acc_3d, data.imu_acc_3d)
    ARRAY_ASSIGN(msg_data.angle_3d, data.angle_3d)
    ARRAY_ASSIGN(msg_data.quaternion, data.quaternion)

    msg_nodes.resize(data.valid_node_count);
    for (size_t i = 0; i < data.valid_node_count; ++i) {
      auto &msg_node = msg_nodes[i];
      auto node = data.nodes[i];
      msg_node.id = node->id;
      msg_node.role = node->role;
      msg_node.dis = node->dis;
      msg_node.fp_rssi = node->fp_rssi;
      msg_node.rx_rssi = node->rx_rssi;
    }

    if (nodeframe2_pub_) {
      nodeframe2_pub_->publish(msg_data);
    }
  });
}

void Init::initNodeFrame3(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLT_ProtocolNodeFrame3;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([this] {
    if (node_ && !nodeframe3_pub_) {
      const auto topic = std::string("nlink_linktrack_nodeframe3");
      nodeframe3_pub_ = node_->create_publisher<nlink_parser::msg::LinktrackNodeframe3>(
          topic, rclcpp::QoS(200));
      TopicAdvertisedTip(node_->get_logger(), topic);
    }
    const auto &data = g_nlt_nodeframe3.result;
    auto &msg_data = g_msg_nodeframe3;
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
      msg_node.fp_rssi = node->fp_rssi;
      msg_node.rx_rssi = node->rx_rssi;
    }

    if (nodeframe3_pub_) {
      nodeframe3_pub_->publish(msg_data);
    }
  });
}

void Init::initNodeFrame4(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLT_ProtocolNodeFrame4;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([this] {
    if (node_ && !nodeframe4_pub_) {
      const auto topic = std::string("nlink_linktrack_nodeframe4");
      nodeframe4_pub_ = node_->create_publisher<nlink_parser::msg::LinktrackNodeframe4>(
          topic, rclcpp::QoS(200));
      TopicAdvertisedTip(node_->get_logger(), topic);
    }
    const auto &data = g_nlt_nodeframe4.result;
    auto &msg_data = g_msg_nodeframe4;
    msg_data.role = data.role;
    msg_data.id = data.id;
    msg_data.local_time = data.local_time;
    msg_data.system_time = data.system_time;
    msg_data.voltage = data.voltage;
    msg_data.tags.resize(data.tag_count);
    for (int i = 0; i < data.tag_count; ++i) {
      auto &msg_tag = msg_data.tags[i];
      auto tag = data.tags[i];
      msg_tag.id = tag->id;
      msg_tag.voltage = tag->voltage;
      msg_tag.anchors.resize(tag->anchor_count);
      for (int j = 0; j < tag->anchor_count; ++j) {
        auto &msg_anchor = msg_tag.anchors[j];
        auto anchor = tag->anchors[j];
        msg_anchor.id = anchor->id;
        msg_anchor.dis = anchor->dis;
      }
    }

    if (nodeframe4_pub_) {
      nodeframe4_pub_->publish(msg_data);
    }
  });
}

void Init::initNodeFrame5(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLT_ProtocolNodeFrame5;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([this] {
    if (node_ && !nodeframe5_pub_) {
      const auto topic = std::string("nlink_linktrack_nodeframe5");
      nodeframe5_pub_ = node_->create_publisher<nlink_parser::msg::LinktrackNodeframe5>(
          topic, rclcpp::QoS(200));
      TopicAdvertisedTip(node_->get_logger(), topic);
    }
    const auto &data = g_nlt_nodeframe5.result;
    auto &msg_data = g_msg_nodeframe5;
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
      msg_node.fp_rssi = node->fp_rssi;
      msg_node.rx_rssi = node->rx_rssi;
    }

    if (nodeframe5_pub_) {
      nodeframe5_pub_->publish(msg_data);
    }
  });
}

void Init::initNodeFrame6(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLT_ProtocolNodeFrame6;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([this] {
    if (node_ && !nodeframe6_pub_) {
      const auto topic = std::string("nlink_linktrack_nodeframe6");
      nodeframe6_pub_ = node_->create_publisher<nlink_parser::msg::LinktrackNodeframe6>(
          topic, rclcpp::QoS(200));
      TopicAdvertisedTip(node_->get_logger(), topic);
    }
    const auto &data = g_nlt_nodeframe6.result;
    auto &msg_data = g_msg_nodeframe6;
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

    if (nodeframe6_pub_) {
      nodeframe6_pub_->publish(msg_data);
    }
  });
}

} // namespace linktrack
