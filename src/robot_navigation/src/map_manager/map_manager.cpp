#include "robot_navigation/map_manager/map_manager.hpp"


MapManager::MapManager()
: inflation_radius_(0.3), obstacle_threshold_(50)
{

}

MapManager::~MapManager()
{

}

nav_msgs::msg::OccupancyGrid MapManager::processMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg)
{
    // 将OccupancyGrid转换为OpenCV图像
    cv::Mat map_image = occupancyGridToCvImage(map_msg);
    
    // 对障碍物进行膨胀处理
    cv::Mat inflated_map = inflateObstacles(map_image, inflation_radius_, map_msg->info.resolution);
    
    // 将处理后的图像转换回OccupancyGrid
    return cvImageToOccupancyGrid(inflated_map, map_msg);
}

nav_msgs::msg::OccupancyGrid MapManager::cvImageToOccupancyGrid(const cv::Mat& map_image, 
                                                                          const nav_msgs::msg::OccupancyGrid::SharedPtr original_map)
{
    nav_msgs::msg::OccupancyGrid map_msg;
    
    // 复制元数据
    map_msg.header = original_map->header;
    map_msg.info = original_map->info;
    
    // 调整数据大小
    map_msg.data.resize(map_image.rows * map_image.cols);
    
    // 填充数据
    for (int i = 0; i < map_image.rows; ++i) {
        for (int j = 0; j < map_image.cols; ++j) {
            int idx = i * map_image.cols + j;
            uchar value = map_image.at<uchar>(i, j);
            
            if (value == 127) {  // 未知区域
                map_msg.data[idx] = -1;
            } else if (value < 100) {  // 障碍物
                map_msg.data[idx] = 100;
            } else {  // 自由区域
                map_msg.data[idx] = 0;
            }
        }
    }
    
    return map_msg;
}

cv::Mat MapManager::occupancyGridToCvImage(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg)
{
    // 创建空白图像
    cv::Mat map_image(map_msg->info.height, map_msg->info.width, CV_8UC1);
    
    // 填充图像数据
    for (unsigned int i = 0; i < map_msg->data.size(); ++i) {
        // 计算对应的图像坐标
        unsigned int row = i / map_msg->info.width;
        unsigned int col = i % map_msg->info.width;
        
        int value = map_msg->data[i];
        if (value == -1) {  // 未知区域
            map_image.at<uchar>(row, col) = 127;  // 灰色表示未知
        } else {
            // 将占用概率映射到像素值
            // 值越大表示越可能是障碍物，所以这里取反
            map_image.at<uchar>(row, col) = static_cast<uchar>(255 - value);
        }
    }
    
    // 二值化处理
    if (obstacle_threshold_ < 0) {
        cv::threshold(map_image, map_image, 0, 255, cv::THRESH_OTSU);
    } else {
        cv::threshold(map_image, map_image, 255 - obstacle_threshold_, 255, cv::THRESH_BINARY);
    }
    
    return map_image;
}

cv::Mat MapManager::inflateObstacles(const cv::Mat& map_image, double inflation_radius, double resolution)
{
    // 计算膨胀半径对应的像素数
    int inflation_pixels = static_cast<int>(inflation_radius / resolution);
    
    // 创建膨胀核
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, 
                                              cv::Size(2 * inflation_pixels + 1, 2 * inflation_pixels + 1));
    
    // 对障碍物进行膨胀处理
    cv::Mat inflated_map;
    cv::dilate(map_image, inflated_map, kernel);
    
    return inflated_map;
}

