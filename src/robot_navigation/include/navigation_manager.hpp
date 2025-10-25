#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include "map_manager.hpp"
#include "global_planner.hpp"

class NavigationNode : public rclcpp::Node
{
public:
    static NavigationNode& getInstance() {
        static NavigationNode instance;
        return instance;
    }

    // 删除拷贝构造函数和赋值操作符
    NavigationNode(const NavigationNode&) = delete;
    NavigationNode& operator=(const NavigationNode&) = delete;

    void setCurMapInfo(const nav_msgs::msg::OccupancyGrid::SharedPtr& map_msg);
    void setNaviStartPose(const geometry_msgs::msg::PoseStamped::SharedPtr& start_msg);
    void setNaviGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr& goal_msg);


private:
    NavigationNode();
    ~NavigationNode() override = default;

    // === 初始化相关函数 ===
    void declareAndGetParameters();
    void initializePlanner() const;
    
    // === 功能类实例 ===
    std::shared_ptr<MapManager> map_manager_ptr_;
    std::shared_ptr<GlobalPlanner> global_planner_ptr_;
    
    // === 数据缓存 ===
    double inflation_radius;    // 膨胀半径
    double obstacle_threshold;  // 障碍物阈值
    double planning_frequency;  // 规划频率
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    geometry_msgs::msg::PoseStamped current_start_;
    geometry_msgs::msg::PoseStamped current_goal_;
    bool has_start_ = false;
    bool has_goal_ = false;

    // === 路径规划 ===
    void planPath();
};
