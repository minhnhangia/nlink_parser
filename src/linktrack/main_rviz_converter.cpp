#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nlink_parser/msg/linktrack_anchorframe0.hpp>
#include <nlink_parser/msg/linktrack_nodeframe1.hpp>
#include <nlink_parser/msg/linktrack_nodeframe2.hpp>
#include <nlink_parser/msg/linktrack_tagframe0.hpp>
#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <map>
#include <sstream>
#include <string>

#include "nutils.h"

namespace {
struct PosePublisher {
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;
  geometry_msgs::msg::PoseStamped msg;
};
} // namespace

class LinktrackRvizConverter : public rclcpp::Node {
public:
  LinktrackRvizConverter() : Node("linktrack_example") {
    frame_id_ = declare_parameter<std::string>("map_frame", "linktrack_map");

    sub_anchorframe0_ = create_subscription<nlink_parser::msg::LinktrackAnchorframe0>(
        "nlink_linktrack_anchorframe0", rclcpp::QoS(10),
        std::bind(&LinktrackRvizConverter::anchorframe0Callback, this,
                  std::placeholders::_1));

    sub_tagframe0_ = create_subscription<nlink_parser::msg::LinktrackTagframe0>(
        "nlink_linktrack_tagframe0", rclcpp::QoS(10),
        std::bind(&LinktrackRvizConverter::tagframe0Callback, this,
                  std::placeholders::_1));

    sub_nodeframe1_ = create_subscription<nlink_parser::msg::LinktrackNodeframe1>(
        "nlink_linktrack_nodeframe1", rclcpp::QoS(10),
        std::bind(&LinktrackRvizConverter::nodeframe1Callback, this,
                  std::placeholders::_1));

    sub_nodeframe2_ = create_subscription<nlink_parser::msg::LinktrackNodeframe2>(
        "nlink_linktrack_nodeframe2", rclcpp::QoS(10),
        std::bind(&LinktrackRvizConverter::nodeframe2Callback, this,
                  std::placeholders::_1));
  }

private:
  void anchorframe0Callback(
      const nlink_parser::msg::LinktrackAnchorframe0::SharedPtr msg) {
    for (const auto &node : msg->nodes) {
      auto id = node.id;
      auto &pose = anchorframe0_poses_[id];
      if (!pose.publisher) {
        std::ostringstream string_stream;
        string_stream << "nlt_anchorframe0_pose_node" << static_cast<int>(id);
        auto topic = string_stream.str();
        pose.publisher = create_publisher<geometry_msgs::msg::PoseStamped>(
            topic, rclcpp::QoS(10));
        pose.msg.header.frame_id = frame_id_;
        pose.msg.pose.orientation.w = 0.0;
        pose.msg.pose.orientation.x = 0.0;
        pose.msg.pose.orientation.y = 0.0;
        pose.msg.pose.orientation.z = 1.0;
        TopicAdvertisedTip(get_logger(), topic);
      }
      auto &msg_pose = pose.msg;
      msg_pose.header.stamp = now();
      msg_pose.pose.position.x = static_cast<double>(node.pos_3d[0]);
      msg_pose.pose.position.y = static_cast<double>(node.pos_3d[1]);
      msg_pose.pose.position.z = static_cast<double>(node.pos_3d[2]);
      pose.publisher->publish(msg_pose);
    }
  }

  void nodeframe1Callback(
      const nlink_parser::msg::LinktrackNodeframe1::SharedPtr msg) {
    for (const auto &node : msg->nodes) {
      auto id = node.id;
      auto &pose = nodeframe1_poses_[id];
      if (!pose.publisher) {
        std::ostringstream string_stream;
        string_stream << "nlt_nodeframe1_pose_node" << static_cast<int>(id);
        auto topic = string_stream.str();
        pose.publisher = create_publisher<geometry_msgs::msg::PoseStamped>(
            topic, rclcpp::QoS(10));
        pose.msg.header.frame_id = frame_id_;
        pose.msg.pose.orientation.w = 0.0;
        pose.msg.pose.orientation.x = 0.0;
        pose.msg.pose.orientation.y = 0.0;
        pose.msg.pose.orientation.z = 1.0;
        TopicAdvertisedTip(get_logger(), topic);
      }
      auto &msg_pose = pose.msg;
      msg_pose.header.stamp = now();
      msg_pose.pose.position.x = static_cast<double>(node.pos_3d[0]);
      msg_pose.pose.position.y = static_cast<double>(node.pos_3d[1]);
      msg_pose.pose.position.z = static_cast<double>(node.pos_3d[2]);
      pose.publisher->publish(msg_pose);
    }
  }

  void tagframe0Callback(
      const nlink_parser::msg::LinktrackTagframe0::SharedPtr msg) {
    if (!tagframe0_pose_.publisher) {
      const std::string topic = "nlt_tagframe0_pose";
      tagframe0_pose_.publisher = create_publisher<geometry_msgs::msg::PoseStamped>(
          topic, rclcpp::QoS(10));
      tagframe0_pose_.msg.header.frame_id = frame_id_;
      TopicAdvertisedTip(get_logger(), topic);
    }
    auto &msg_pose = tagframe0_pose_.msg;
    msg_pose.header.stamp = now();
    msg_pose.pose.orientation.w = static_cast<double>(msg->quaternion[0]);
    msg_pose.pose.orientation.x = static_cast<double>(msg->quaternion[1]);
    msg_pose.pose.orientation.y = static_cast<double>(msg->quaternion[2]);
    msg_pose.pose.orientation.z = static_cast<double>(msg->quaternion[3]);
    msg_pose.pose.position.x = static_cast<double>(msg->pos_3d[0]);
    msg_pose.pose.position.y = static_cast<double>(msg->pos_3d[1]);
    msg_pose.pose.position.z = static_cast<double>(msg->pos_3d[2]);
    tagframe0_pose_.publisher->publish(msg_pose);
  }

  void nodeframe2Callback(
      const nlink_parser::msg::LinktrackNodeframe2::SharedPtr msg) {
    if (!nodeframe2_pose_.publisher) {
      const std::string topic = "nlt_nodeframe2_pose";
      nodeframe2_pose_.publisher = create_publisher<geometry_msgs::msg::PoseStamped>(
          topic, rclcpp::QoS(10));
      nodeframe2_pose_.msg.header.frame_id = frame_id_;
      TopicAdvertisedTip(get_logger(), topic);
    }
    auto &msg_pose = nodeframe2_pose_.msg;
    msg_pose.header.stamp = now();
    msg_pose.pose.orientation.w = static_cast<double>(msg->quaternion[0]);
    msg_pose.pose.orientation.x = static_cast<double>(msg->quaternion[1]);
    msg_pose.pose.orientation.y = static_cast<double>(msg->quaternion[2]);
    msg_pose.pose.orientation.z = static_cast<double>(msg->quaternion[3]);
    msg_pose.pose.position.x = static_cast<double>(msg->pos_3d[0]);
    msg_pose.pose.position.y = static_cast<double>(msg->pos_3d[1]);
    msg_pose.pose.position.z = static_cast<double>(msg->pos_3d[2]);
    nodeframe2_pose_.publisher->publish(msg_pose);
  }

  std::string frame_id_;
  std::map<uint8_t, PosePublisher> anchorframe0_poses_;
  std::map<uint8_t, PosePublisher> nodeframe1_poses_;
  PosePublisher tagframe0_pose_;
  PosePublisher nodeframe2_pose_;

  rclcpp::Subscription<nlink_parser::msg::LinktrackAnchorframe0>::SharedPtr
      sub_anchorframe0_;
  rclcpp::Subscription<nlink_parser::msg::LinktrackTagframe0>::SharedPtr
      sub_tagframe0_;
  rclcpp::Subscription<nlink_parser::msg::LinktrackNodeframe1>::SharedPtr
      sub_nodeframe1_;
  rclcpp::Subscription<nlink_parser::msg::LinktrackNodeframe2>::SharedPtr
      sub_nodeframe2_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LinktrackRvizConverter>());
  rclcpp::shutdown();
  return 0;
}
