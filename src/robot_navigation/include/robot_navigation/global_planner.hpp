#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "motion_planning/PathPlannerStrategy.hpp"

class GlobalPlanner
{
public:
    void setPlanningFrequency(double frequency) { planning_frequency_ = frequency; }

    // 设置策略算法
    void setPlanner(std::shared_ptr<PathPlannerStrategy> planner) {
        planner_ = planner;
    }

    void setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& map) {
        current_map_ = map;
    }

    nav_msgs::msg::Path planPath(const geometry_msgs::msg::PoseStamped& start, 
                                 const geometry_msgs::msg::PoseStamped& goal)
    {
        if (!current_map_ || !planner_) {
            RCLCPP_ERROR(rclcpp::get_logger("GlobalPlanner"), "No map or planner not set.");
            return nav_msgs::msg::Path();
        }

        return planner_->plan(current_map_, start, goal);
    }

private:
    double planning_frequency_;
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    std::shared_ptr<PathPlannerStrategy> planner_;
};

