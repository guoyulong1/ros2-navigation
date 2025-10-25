#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
// #include <string__struct.hpp>


class CommManager : public rclcpp::Node
{
public:
    static CommManager& Instance() {
        static CommManager instance("comm_manager");
        return instance;
    }
    CommManager(const CommManager&) = delete;
    CommManager& operator=(const CommManager&) = delete;

    void initialize();

    void publishNaviPath(const nav_msgs::msg::Path& path) const;
    void publishProcessedMap(const nav_msgs::msg::OccupancyGrid& map) const;


private:
    explicit CommManager(const std::string& node_name) : Node(node_name) {}

    void initPublishers();
    void initSubscribers();

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr& map_msg);
    void naviInitialPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr& initial_pose_msg);
    void naviGoalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr& goal_pose_msg);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr navi_goal_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr navi_initial_pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr processed_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr navi_path_pub_;
};