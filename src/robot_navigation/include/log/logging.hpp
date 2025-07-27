#pragma once
#include <rclcpp/rclcpp.hpp>

#define LOG_INFO(...)  RCLCPP_INFO(rclcpp::get_logger("navigation"), __VA_ARGS__)
#define LOG_WARN(...)  RCLCPP_WARN(rclcpp::get_logger("navigation"), __VA_ARGS__)
#define LOG_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("navigation"), __VA_ARGS__)
