#include "motion_planning/a_star_planner.hpp"
#include "utils/logger.hpp"

AStarPlanner::AStarPlanner(const int distance, const int OccupyThresh)
{
    LOG_INFO("distance:{},occupiedThresh:{}", distance, OccupyThresh);
    distance_type_ = distance;
    occupy_thresh_ = OccupyThresh;
}

AStarPlanner::~AStarPlanner()
{
    cleanup();
}

nav_msgs::msg::Path AStarPlanner::plan(
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal)
{
    nav_msgs::msg::Path path;
    
    if (!map || map->data.empty()) {
        LOG_ERROR("Invalid map");
        return path;
    }

    const int width = map->info.width;
    const int height = map->info.height;
    const double resolution = map->info.resolution;
    const double origin_x = map->info.origin.position.x;
    const double origin_y = map->info.origin.position.y;

    cv::Mat map_image(height, width, CV_8UC1);
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            const int i = x +  (height - 1 - y) * width;
            const int val = map->data[i];
            map_image.at<uchar>(y, x) = val < 0 || val > occupy_thresh_ ? 0 : 255;
        }
    }
    cv::imwrite("map.jpg", map_image);

    initAstar(map_image);

    auto worldToMap = [&](const double wx, const double wy) -> cv::Point {
        const int mx = static_cast<int>((wx - origin_x) / resolution);
        const int my = static_cast<int>((wy - origin_y) / resolution);
        return cv::Point(mx, height - 1 - my);  
    };

    start_point_ = worldToMap(start.pose.position.x, start.pose.position.y);
    target_point_ = worldToMap(goal.pose.position.x, goal.pose.position.y);

    // Check if start/goal are valid
    if (!isValid(start_point_) || !isValid(target_point_)) {
        LOG_ERROR("Invalid start/goal position");
        return path;
    }

    if (!isTraversable(start_point_) || !isTraversable(target_point_)) {
        LOG_ERROR("Start or goal point is in obstacle");
        return path;
    }

    // Path Planning
    LOG_INFO("begin find path");
    Node* end_node = findPath();
    LOG_INFO("find path success");

    // Path Reconstruct
    std::vector<cv::Point> pixel_path;
    reconstructPath(end_node, pixel_path);

    for (const auto& pt : pixel_path)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = map->header;

        const double wx = pt.x * resolution + origin_x;
        const double wy = (height - 1 - pt.y) * resolution + origin_y;

        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;

        path.poses.push_back(pose);
    }

    LOG_INFO("Planned path with {} points", path.poses.size());

    return path;
}

void AStarPlanner::initAstar(const cv::Mat& map)
{
    map_ = map.clone();
    processMap();
}

void AStarPlanner::processMap()
{
    const int width = map_.cols;
    const int height = map_.rows;
    cv::Mat processed_map = map_.clone();

    // Transform RGB to gray image
    if(processed_map.channels() == 3)
    {
        cvtColor(processed_map.clone(), processed_map, cv::COLOR_BGR2GRAY);
    }

    // Binarize
    if(occupy_thresh_ < 0)
    {
        threshold(processed_map.clone(), processed_map, 0, 255, cv::THRESH_OTSU);
    } else
    {
        threshold(processed_map.clone(), processed_map, occupy_thresh_, 255, cv::THRESH_BINARY);
    }

    // Initial label_map_
    label_map_ = cv::Mat::zeros(height, width, CV_8UC1);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            label_map_.at<uchar>(y, x) = processed_map.at<uchar>(y, x) == 0 ? obstacle : free;
        }
    }

    // 调试信息
    int obstacle_count = 0;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (label_map_.at<uchar>(y, x) == obstacle) obstacle_count++;
        }
    }
    LOG_INFO("Map processed - Obstacles: {}, Free: {}", obstacle_count, width * height - obstacle_count);
}

AStarPlanner::Node* AStarPlanner::findPath()
{
    cv::Mat search_label_map = label_map_.clone();

    const std::vector<cv::Point> neighbors = {
        {-1, -1}, {-1, 0}, {-1, 1},
        {0, -1},           {0, 1},
        {1, -1},  {1, 0},  {1, 1}
    };
    // cv::Mat neighbors(8, 2, CV_8S, neighbor8);

    cleanup();

    // Add start_point_ to OpenList
    auto start_node = new Node(start_point_);
    start_node->H = calculateHeuristic(start_point_, target_point_);
    start_node->F = start_node->H;

    open_list_.push(std::make_pair(start_node->F, start_node->point));
    open_dict_[pointToIndex(start_point_)] = start_node;
    all_nodes_.push_back(start_node);
    search_label_map.at<uchar>(start_point_.y, start_point_.x) = inOpenList;

    while(!open_list_.empty())
    {

        // Find the node with least F value
        cv::Point current_point = open_list_.top().second;
        open_list_.pop();

        int current_index = pointToIndex(current_point);
        auto it = open_dict_.find(current_index);
        if (it == open_dict_.end()) {
            continue; // 节点已被处理
        }

        Node* current_node = it->second;
        open_dict_.erase(it);
        search_label_map.at<uchar>(current_point.y, current_point.x) = inCloseList;

        // Determine whether arrive the target point
        if (current_point == target_point_) {
            return current_node;
        }

        // Traversal the neighborhood
        for(const auto& d : neighbors)
        {
            const int nx = current_point.x + d.x;
            const int ny = current_point.y + d.y;
            cv::Point neighbor_point(nx, ny);

            if (!isValid(neighbor_point)) {
                continue;
            }

            if (label_map_.at<uchar>(neighbor_point.y, neighbor_point.x) == obstacle) {
                continue;
            }

            const uchar neighbor_search_label = search_label_map.at<uchar>(ny, nx);
            if (neighbor_search_label == obstacle) {
                continue;
            }

            if (std::abs(d.x) == 1 && std::abs(d.y) == 1) {
                cv::Point horz(current_point.x + d.x, current_point.y);
                cv::Point vert(current_point.x, current_point.y + d.y);

                if (!isValid(horz) || !isValid(vert) ||
                    label_map_.at<uchar>(horz.y, horz.x) == obstacle ||
                    label_map_.at<uchar>(vert.y, vert.x) == obstacle) {
                    continue;
                    }
            }

            const int move_cost = (std::abs(d.x) == 1 && std::abs(d.y) == 1) ? COST_DIAGONAL : COST_STRAIGHT;
            const int new_g = current_node->G + move_cost;
            int neighbor_index = pointToIndex(neighbor_point);

            if(neighbor_search_label == free || neighbor_search_label == inOpenList)
            {
                // Update the G, H, F value of node
                if(neighbor_search_label == free)
                {
                    const auto neighbor_node = new Node(neighbor_point);
                    neighbor_node->parent = current_node;
                    neighbor_node->G = new_g;
                    neighbor_node->H = calculateHeuristic(neighbor_point, target_point_);
                    neighbor_node->F = neighbor_node->G + neighbor_node->H;
                    open_list_.push(std::make_pair(neighbor_node->F, neighbor_node->point));
                    open_dict_[neighbor_index] = neighbor_node;
                    all_nodes_.push_back(neighbor_node);
                    search_label_map.at<uchar>(ny, nx) = inOpenList;
                }
                else
                {
                    // Find the node
                    if (Node* existing_node = open_dict_[neighbor_index]; new_g < existing_node->G)
                    {
                        existing_node->G = new_g;
                        existing_node->F = existing_node->G + existing_node->H;
                        existing_node->parent = current_node;
                        // 由于优先队列不支持更新，重新插入更新后的节点
                        open_list_.push(std::make_pair(existing_node->F, existing_node->point));
                    }
                }
            }
        }
    }

    return nullptr; // Can not find a valid path
}

void AStarPlanner::reconstructPath(const Node* end_node, std::vector<cv::Point>& path)
{
    path.clear();

    // 从终点回溯到起点
    const Node* current = end_node;
    while (current != nullptr) {
        path.push_back(current->point);
        current = current->parent;
    }

    // 反转路径，使其从起点到终点
    std::reverse(path.begin(), path.end());
}

void AStarPlanner::cleanup()
{
    // 清理所有节点内存
    for (const Node* node : all_nodes_) {
        delete node;
    }
    all_nodes_.clear();
    open_dict_.clear();

    // 清空优先队列
    while (!open_list_.empty()) {
        open_list_.pop();
    }
}

int AStarPlanner::calculateHeuristic(const cv::Point& from, const cv::Point& to) const
{
    const int dx = abs(from.x - to.x);
    const int dy = abs(from.y - to.y);

    switch (distance_type_) {
    case 1: // Euclidean
        return static_cast<int>(COST_STRAIGHT * std::sqrt(dx * dx + dy * dy));
    case 2: // Manhattan
        return COST_STRAIGHT * (dx + dy);
    case 3: // Chebyshev
        return COST_STRAIGHT * std::max(dx, dy);
    case 4: // Diagonal
        return COST_STRAIGHT * (dx + dy) + (COST_DIAGONAL - 2 * COST_STRAIGHT) * std::min(dx, dy);
    default:
        return COST_STRAIGHT * (dx + dy); // 默认使用曼哈顿距离
    }
}