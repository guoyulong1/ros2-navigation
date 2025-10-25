#include "navigation_manager.hpp"
#include "motion_planning/a_star_planner.hpp"
#include "utils/logger.hpp"
#include "comm_manager.hpp"

NavigationNode::NavigationNode()
: Node("navigation_node")
{
    map_manager_ptr_ = std::make_shared<MapManager>();
    global_planner_ptr_ = std::make_shared<GlobalPlanner>();
    declareAndGetParameters();    // 声明并获取参数
    initializePlanner();          // 初始化规划器
    LOG_INFO("NavigationNode initialized");
}

void NavigationNode::declareAndGetParameters()
{
    this->declare_parameter("inflation_radius", 0.3);
    this->declare_parameter("obstacle_threshold", 50.0);
    this->declare_parameter("planning_frequency", 1.0);

   
    this->get_parameter("inflation_radius", inflation_radius);
    this->get_parameter("obstacle_threshold", obstacle_threshold);
    this->get_parameter("planning_frequency", planning_frequency);

    // 将参数应用到 map_manager_ / global_planner_ 的接口
    map_manager_ptr_->setInflationRadius(inflation_radius);
    map_manager_ptr_->setObstacleThreshold(obstacle_threshold);
    global_planner_ptr_->setPlanningFrequency(planning_frequency);
}

void NavigationNode::initializePlanner() const
{
    double inflation_radius = 0.3;
    double obstacle_threshold = 50.0;

    this->get_parameter("inflation_radius", inflation_radius);
    this->get_parameter("obstacle_threshold", obstacle_threshold);

    global_planner_ptr_->setPlanner(std::make_shared<AStarPlanner>(1, obstacle_threshold, inflation_radius));
}

void NavigationNode::setCurMapInfo(const nav_msgs::msg::OccupancyGrid::SharedPtr& map_msg)
{
    current_map_ = map_msg;

    global_planner_ptr_->setMap(map_msg);

    // 处理地图
    auto processed_map = map_manager_ptr_->processMap(map_msg);
    
    // 发布处理后的地图
    CommManager::Instance().publishProcessedMap(processed_map);

    // auto processed_map_ptr = std::make_shared<nav_msgs::msg::OccupancyGrid>(processed_map);
    // global_planner_.setMap(processed_map_ptr);
    
    // 如果有起点和终点，尝试规划路径
    if (has_start_ && has_goal_) {
        planPath();
    }
}

void NavigationNode::setNaviStartPose(const geometry_msgs::msg::PoseStamped::SharedPtr& start_msg)
{
    current_start_ = *start_msg;
    has_start_ = true;
    
    // 如果有地图和终点，尝试规划路径
    if (current_map_ && has_goal_) {
        planPath();
    }
}

void NavigationNode::setNaviGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr& goal_msg)
{
    current_goal_ = *goal_msg;
    has_goal_ = true;
    
    // 如果有地图和起点，尝试规划路径
    if (current_map_ && has_start_) {
        planPath();
    }
}

void NavigationNode::planPath()
{
    if (!current_map_ || !has_start_ || !has_goal_) {
        LOG_WARN("Cannot plan path: missing map, start, or goal");
        return;
    }
    
    // 使用全局规划器规划路径
    auto path = global_planner_ptr_->planPath(current_start_, current_goal_);
    
    // 发布路径
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "map";
    CommManager::Instance().publishNaviPath(path);
    
    LOG_INFO("Path planned with {} points", path.poses.size());
}
