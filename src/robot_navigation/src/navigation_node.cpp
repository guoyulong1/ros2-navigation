#include "robot_navigation/navigation_node.hpp"
#include "robot_navigation/planners/a_star_planner.hpp"

NavigationNode::NavigationNode() : Node("navigation_node"), has_start_(false), has_goal_(false)
{
    // 声明和获取参数
    this->declare_parameter("inflation_radius", 0.3);
    this->declare_parameter("obstacle_threshold", 50.0);
    this->declare_parameter("planning_frequency", 1.0);
    
    double inflation_radius, obstacle_threshold, planning_frequency;
    this->get_parameter("inflation_radius", inflation_radius);
    this->get_parameter("obstacle_threshold", obstacle_threshold);
    this->get_parameter("planning_frequency", planning_frequency);
    
    // 配置功能类
    map_manager_.setInflationRadius(inflation_radius);
    map_manager_.setObstacleThreshold(obstacle_threshold);
    global_planner_.setPlanningFrequency(planning_frequency);
    
    // 创建订阅者和发布者
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 10, std::bind(&NavigationNode::mapCallback, this, std::placeholders::_1));
    
    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            geometry_msgs::msg::PoseStamped start;
            start.header = msg->header;
            start.pose = msg->pose.pose;
            RCLCPP_INFO(this->get_logger(), "Initial pose received from RViz");

            auto start_msg = std::make_shared<geometry_msgs::msg::PoseStamped>(start);
            this->startCallback(start_msg);
        });
    
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10, std::bind(&NavigationNode::goalCallback, this, std::placeholders::_1));
    
    processed_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("processed_map", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 10);

    global_planner_.setPlanner(std::make_shared<AStarPlanner>(1,obstacle_threshold,inflation_radius));
    
    RCLCPP_INFO(this->get_logger(), "Navigation Node initialized");
}

NavigationNode::~NavigationNode()
{
}

void NavigationNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg)
{
    current_map_ = map_msg;

    global_planner_.setMap(map_msg);

    // 处理地图
    auto processed_map = map_manager_.processMap(map_msg);
    
    // 发布处理后的地图
    processed_map_pub_->publish(processed_map);

    // auto processed_map_ptr = std::make_shared<nav_msgs::msg::OccupancyGrid>(processed_map);
    // global_planner_.setMap(processed_map_ptr);
    
    // 如果有起点和终点，尝试规划路径
    if (has_start_ && has_goal_) {
        planPath();
    }
}

void NavigationNode::startCallback(const geometry_msgs::msg::PoseStamped::SharedPtr start_msg)
{
    current_start_ = *start_msg;
    has_start_ = true;
    RCLCPP_INFO(this->get_logger(), "Start pose received");
    
    // 如果有地图和终点，尝试规划路径
    if (current_map_ && has_goal_) {
        planPath();
    }
}

void NavigationNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg)
{
    current_goal_ = *goal_msg;
    has_goal_ = true;
    RCLCPP_INFO(this->get_logger(), "Goal pose received");
    
    // 如果有地图和起点，尝试规划路径
    if (current_map_ && has_start_) {
        planPath();
    }
}

void NavigationNode::planPath()
{
    if (!current_map_ || !has_start_ || !has_goal_) {
        RCLCPP_WARN(this->get_logger(), "Cannot plan path: missing map, start, or goal");
        return;
    }
    
    // 使用全局规划器规划路径
    auto path = global_planner_.planPath(current_start_, current_goal_);
    
    // 设置路径的header
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "map";
    
    // 发布路径
    path_pub_->publish(path);
    
    RCLCPP_INFO(this->get_logger(), "Path planned with %zu points", path.poses.size());
}
