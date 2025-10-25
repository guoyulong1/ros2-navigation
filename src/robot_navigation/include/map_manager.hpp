#pragma once

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <image_transport/image_transport.hpp>

class MapManager
{
public:
    MapManager();
    ~MapManager();

    void setInflationRadius(const double radius) { inflation_radius_ = radius; }
    void setObstacleThreshold(const double threshold) { obstacle_threshold_ = threshold; }
    nav_msgs::msg::OccupancyGrid processMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& map_msg) const;

private:
    cv::Mat occupancyGridToCvImage(const nav_msgs::msg::OccupancyGrid::SharedPtr& map_msg) const;
    static cv::Mat inflateObstacles(const cv::Mat& map_image, double inflation_radius, double resolution);
    static nav_msgs::msg::OccupancyGrid cvImageToOccupancyGrid(const cv::Mat& map_image,
                                                               const nav_msgs::msg::OccupancyGrid::SharedPtr& original_map);


private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr processed_map_pub_;
    image_transport::Publisher map_image_pub_;

    std::shared_ptr<image_transport::ImageTransport> it_;

    double inflation_radius_;
    double obstacle_threshold_;
};