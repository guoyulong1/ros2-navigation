#include "comm_manager.hpp"
#include "navigation_manager.hpp"
#include "utils/logger.hpp"
#include "navigation_manager.hpp"

void CommManager::initialize()
{
    // 初始化通信管理器的相关设置
    initPublishers();
    initSubscribers();
    LOG_INFO("CommManager initialized");
}

void CommManager::initPublishers()
{
    // 初始化发布者
    processed_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("processed_map", 10);
    navi_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 10);
}

void CommManager::initSubscribers()
{
    // 初始化订阅者 - 使用正确的回调签名
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 10,
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            this->mapCallback(msg);
        });

    navi_initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            geometry_msgs::msg::PoseStamped start;
            start.header = msg->header;
            start.pose = msg->pose.pose;

            auto start_msg = std::make_shared<geometry_msgs::msg::PoseStamped>(start);
            this->naviInitialPoseCallback(start_msg);
        });

    navi_goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            this->naviGoalPoseCallback(msg);
        });
}

void CommManager::publishNaviPath(const nav_msgs::msg::Path& path) const
{
    navi_path_pub_->publish(path);
}

void CommManager::publishProcessedMap(const nav_msgs::msg::OccupancyGrid& map) const
{
    processed_map_pub_->publish(map);
}

void CommManager::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr& map_msg)
{
    // 处理接收到的地图消息
    LOG_INFO("Map received in CommManager");
    NavigationNode::getInstance().setCurMapInfo(map_msg);
    
}

void CommManager::naviInitialPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr& initial_pose_msg)
{
    // 处理接收到的初始位姿消息
    LOG_INFO("Initial pose received in CommManager");
    NavigationNode::getInstance().setNaviStartPose(initial_pose_msg);
}

void CommManager::naviGoalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr& goal_pose_msg)
{
    // 处理接收到的目标位姿消息
    LOG_INFO("Goal pose received in CommManager");
    NavigationNode::getInstance().setNaviGoalPose(goal_pose_msg);
}