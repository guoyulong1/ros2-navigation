#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class PathPlannerStrategy
{
public:
    virtual nav_msgs::msg::Path plan(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal) = 0;
    
    virtual ~PathPlannerStrategy() = default;
};