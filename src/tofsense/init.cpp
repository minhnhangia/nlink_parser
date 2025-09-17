#include "init.h"

#include <chrono>
#include <string>

#include "nlink_protocol.h"
#include "nlink_unpack/nlink_tofsense_frame0.h"
#include "nlink_unpack/nlink_utils.h"
#include "nutils.h"

class NTS_ProtocolFrame0 : public NLinkProtocol {
public:
  NTS_ProtocolFrame0();

protected:
  void UnpackFrameData(const uint8_t *data) override;
};

NTS_ProtocolFrame0::NTS_ProtocolFrame0()
    : NLinkProtocol(true, g_nts_frame0.fixed_part_size,
                    {g_nts_frame0.frame_header, g_nts_frame0.function_mark}) {}

void NTS_ProtocolFrame0::UnpackFrameData(const uint8_t *data) {
  g_nts_frame0.UnpackData(data, length());
}

namespace tofsense {

nlink_parser::msg::TofsenseFrame0 g_msg_frame0;

#pragma pack(push, 1)
struct {
  char header[2]{0x57, 0x10};
  uint8_t reserved0[2]{0xff, 0xff};
  uint8_t id{};
  uint8_t reserved1[2]{0xff, 0xff};
  uint8_t checkSum{};
} g_command_read;
#pragma pack(pop)

Init::Init(NProtocolExtracter *protocol_extraction, serial::Serial *serial,
           const rclcpp::Node::SharedPtr &node)
    : node_(node), serial_(serial) {
  if (node_ && serial_) {
    is_inquire_mode_ = node_->declare_parameter<bool>("inquire_mode", true);
  } else {
    is_inquire_mode_ = false;
  }

  InitFrame0(protocol_extraction);
}

void Init::InitFrame0(NProtocolExtracter *protocol_extraction) {
  static auto protocol_frame0_ = new NTS_ProtocolFrame0;
  protocol_extraction->AddProtocol(protocol_frame0_);
  protocol_frame0_->SetHandleDataCallback([this] {
    if (node_) {
      if (is_inquire_mode_ && !cascade_pub_) {
        const auto topic = std::string("nlink_tofsense_cascade");
        cascade_pub_ =
            node_->create_publisher<nlink_parser::msg::TofsenseCascade>(
                topic, rclcpp::QoS(50));
        TopicAdvertisedTip(node_->get_logger(), topic);
      }
      if (!is_inquire_mode_ && !frame0_pub_) {
        const auto topic = std::string("nlink_tofsense_frame0");
        frame0_pub_ =
            node_->create_publisher<nlink_parser::msg::TofsenseFrame0>(
                topic, rclcpp::QoS(50));
        TopicAdvertisedTip(node_->get_logger(), topic);
      }
    }

    const auto &data = g_nts_frame0.result;

    g_msg_frame0.id = data.id;
    g_msg_frame0.system_time = data.system_time;
    g_msg_frame0.dis = data.dis;
    g_msg_frame0.dis_status = data.dis_status;
    g_msg_frame0.signal_strength = data.signal_strength;
    g_msg_frame0.range_precision = data.range_precision;

    if (is_inquire_mode_) {
      frame0_map_[data.id] = g_msg_frame0;
    } else if (frame0_pub_) {
      frame0_pub_->publish(g_msg_frame0);
    }
  });

  if (is_inquire_mode_ && node_ && serial_) {
    timer_scan_ = node_->create_wall_timer(
        std::chrono::duration<double>(1.0 / frequency_), [this]() {
          frame0_map_.clear();
          node_index_ = 0;
          if (timer_read_) {
            timer_read_->reset();
          }
        });
    timer_read_ = node_->create_wall_timer(std::chrono::duration<double>(0.006),
                                           [this]() {
                                             if (node_index_ >= 8) {
                                               if (!frame0_map_.empty() &&
                                                   cascade_pub_) {
                                                 nlink_parser::msg::TofsenseCascade
                                                     msg_cascade;
                                                 for (const auto &msg : frame0_map_) {
                                                   msg_cascade.nodes.push_back(
                                                       msg.second);
                                                 }
                                                 cascade_pub_->publish(msg_cascade);
                                               }
                                               if (timer_read_) {
                                                 timer_read_->cancel();
                                               }
                                             } else {
                                               g_command_read.id = node_index_;
                                               auto data = reinterpret_cast<uint8_t *>(
                                                   &g_command_read);
                                               NLink_UpdateCheckSum(data,
                                                                    sizeof(g_command_read));
                                               serial_->write(data,
                                                              sizeof(g_command_read));
                                               ++node_index_;
                                             }
                                           });
    timer_read_->cancel();
  }
}

} // namespace tofsense
