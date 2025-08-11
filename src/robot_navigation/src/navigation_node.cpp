#include "robot_navigation/navigation_node.hpp"
#include "robot_navigation/motion_planning/a_star_planner.hpp"
#include "log/logging.hpp"

NavigationNode::NavigationNode()
: Node("navigation_node")
{
    map_manager_ptr_ = std::make_shared<MapManager>();
    global_planner_ptr_ = std::make_shared<GlobalPlanner>();
    declareAndGetParameters();    // 声明并获取参数
    configureFunctionalModules(); // 配置功能模块
    createSubscribers();          // 创建订阅者
    createPublishers();           // 创建发布者
    initializePlanner();          // 初始化规划器
    logStartupInfo();             // 打印启动信息
}

NavigationNode::~NavigationNode()
{
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

void NavigationNode::configureFunctionalModules()
{
    // 如果后续有更多复杂功能类（如局部规划器、控制器），可以在这里初始化
}

void NavigationNode::createSubscribers()
{
    // 创建订阅者
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
    
}
void NavigationNode::createPublishers()
{
    // 创建发布者
    processed_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("processed_map", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 10);
}

void NavigationNode::initializePlanner()
{
    double inflation_radius = 0.3;
    double obstacle_threshold = 50.0;

    this->get_parameter("inflation_radius", inflation_radius);
    this->get_parameter("obstacle_threshold", obstacle_threshold);

    global_planner_ptr_->setPlanner(std::make_shared<AStarPlanner>(1, obstacle_threshold, inflation_radius));
}

void NavigationNode::logStartupInfo()
{
    LOG_INFO("Navigation Node initialized and ready.");
}

void NavigationNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg)
{
    current_map_ = map_msg;

    global_planner_ptr_->setMap(map_msg);

    // 处理地图
    auto processed_map = map_manager_ptr_->processMap(map_msg);
    
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
    LOG_INFO("Start pose received");
    
    // 如果有地图和终点，尝试规划路径
    if (current_map_ && has_goal_) {
        planPath();
    }
}

void NavigationNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg)
{
    current_goal_ = *goal_msg;
    has_goal_ = true;
    LOG_INFO("Goal pose received");
    
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
    
    // 设置路径的header
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "map";
    
    // 发布路径
    path_pub_->publish(path);
    
    LOG_INFO("Path planned with {} points", path.poses.size());
}
