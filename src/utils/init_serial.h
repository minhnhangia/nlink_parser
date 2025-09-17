#ifndef INITSERIAL_H
#define INITSERIAL_H

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

void initSerial(serial::Serial *serial, const rclcpp::Node::SharedPtr &node);

#endif // INITSERIAL_H
