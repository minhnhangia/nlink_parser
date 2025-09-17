#include "nutils.h"

#include <rclcpp/rclcpp.hpp>

void TopicAdvertisedTip(const rclcpp::Logger &logger, const std::string &topic) {
  RCLCPP_INFO(logger,
              "%s has been advertised, use 'ros2 topic echo /%s' to view the data",
              topic.c_str(), topic.c_str());
}
