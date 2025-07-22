#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "map_manager.hpp"
#include "global_planner.hpp"

class NavigationNode : public rclcpp::Node
{
public:
    NavigationNode();
    ~NavigationNode();

private:
    // 功能类实例
    MapManager map_manager_;
    GlobalPlanner global_planner_;
    
    // ROS2接口
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr processed_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    // 数据缓存
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    geometry_msgs::msg::PoseStamped current_start_;
    geometry_msgs::msg::PoseStamped current_goal_;
    bool has_start_;
    bool has_goal_;
    
    // 回调函数
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg);
    void startCallback(const geometry_msgs::msg::PoseStamped::SharedPtr start_msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg);
    
    // 规划路径
    void planPath();
};
