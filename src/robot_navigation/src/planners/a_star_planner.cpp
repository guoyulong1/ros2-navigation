
#include "robot_navigation/planners/a_star_planner.hpp"



AStarPlanner::AStarPlanner(const int distance, const int OccupyThresh, const int InflateRadius)
{
    Distance_ = distance;
    OccupyThresh_ = OccupyThresh;
    InflateRadius_ = InflateRadius;
}

nav_msgs::msg::Path AStarPlanner::plan(
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal)
{
    nav_msgs::msg::Path Path;
    
    if (!map || map->data.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("AStarPlanner"), "Invalid map");
        return Path;
    }
    
    int width = map->info.width;
    int height = map->info.height;
    double resolution = map->info.resolution;
    double origin_x = map->info.origin.position.x;
    double origin_y = map->info.origin.position.y;

    cv::Mat map_image(height, width, CV_8UC1);
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            int i = x + (height - 1 - y) * width;
            int val = map->data[i];
            map_image.at<uchar>(y, x) = (val < 0 || val > 50) ? 0 : 255;
        }
    }

    InitAstar(map_image);

    // Check if start/goal are valid
    if (!isValid(startPoint) || !isValid(targetPoint)) {
        RCLCPP_ERROR(rclcpp::get_logger("AStarPlanner"), "Invalid start/goal position");
        return Path;
    }

    auto worldToMap = [&](double wx, double wy) -> cv::Point {
        int mx = static_cast<int>((wx - origin_x) / resolution);
        int my = static_cast<int>((wy - origin_y) / resolution);
        return cv::Point(mx, height - 1 - my);  
    };

    startPoint = worldToMap(start.pose.position.x, start.pose.position.y);
    targetPoint = worldToMap(goal.pose.position.x, goal.pose.position.y);
    std::vector<cv::Point> pixel_path;
    // Path Planning
    Node* TailNode = FindPath();
    GetPath(TailNode, pixel_path);

    for (const auto& pt : pixel_path)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = map->header;

        double wx = pt.x * resolution + origin_x;
        double wy = (height - 1 - pt.y) * resolution + origin_y;

        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;

        Path.poses.push_back(pose);
    }

    RCLCPP_INFO(rclcpp::get_logger("AStarPlanner"), "Planned path with %lu points", Path.poses.size());

    return Path;
}

void AStarPlanner::InitAstar(cv::Mat& _Map)
{
    cv::Mat Mask;
    char neighbor8[8][2] = {
            {-1, -1}, {-1, 0}, {-1, 1},
            {0, -1},            {0, 1},
            {1, -1},   {1, 0},  {1, 1}
    };

    Map = _Map;
    neighbor = cv::Mat(8, 2, CV_8S, neighbor8).clone();

    MapProcess(Mask);
}

void AStarPlanner::MapProcess(cv::Mat& Mask)
{
    int width = Map.cols;
    int height = Map.rows;
    cv::Mat _Map = Map.clone();

    // Transform RGB to gray image
    if(_Map.channels() == 3)
    {
        cvtColor(_Map.clone(), _Map, cv::COLOR_BGR2GRAY);
    }

    // Binarize
    if(OccupyThresh_ < 0)
    {
        threshold(_Map.clone(), _Map, 0, 255, cv::THRESH_OTSU);
    } else
    {
        threshold(_Map.clone(), _Map, OccupyThresh_, 255, cv::THRESH_BINARY);
    }

    // Inflate
    cv::Mat src = _Map.clone();
    if(InflateRadius_ > 0)
    {
        cv::Mat se = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * InflateRadius_, 2 * InflateRadius_));
        erode(src, _Map, se);
    }

    // Get mask
    bitwise_xor(src, _Map, Mask);

    // Initial LabelMap
    LabelMap = cv::Mat::zeros(height, width, CV_8UC1);
    for(int y=0;y<height;y++)
    {
        for(int x=0;x<width;x++)
        {
            if(_Map.at<uchar>(y, x) == 0)
            {
                LabelMap.at<uchar>(y, x) = obstacle;
            }
            else
            {
                LabelMap.at<uchar>(y, x) = free;
            }
        }
    }
}

AStarPlanner::Node* AStarPlanner::FindPath()
{
    int width = Map.cols;
    int height = Map.rows;
    cv::Mat _LabelMap = LabelMap.clone();

    // Add startPoint to OpenList
    Node* startPointNode = new Node(startPoint);
    OpenList.push(std::pair<int, cv::Point>(startPointNode->F, startPointNode->point));
    int index = point2index(startPointNode->point);
    OpenDict[index] = startPointNode;
    _LabelMap.at<uchar>(startPoint.y, startPoint.x) = inOpenList;

    while(!OpenList.empty())
    {
        // Find the node with least F value
        cv::Point CurPoint = OpenList.top().second;
        OpenList.pop();
        int index = point2index(CurPoint);
        Node* CurNode = OpenDict[index];
        OpenDict.erase(index);

        int curX = CurPoint.x;
        int curY = CurPoint.y;
        _LabelMap.at<uchar>(curY, curX) = inCloseList;

        // Determine whether arrive the target point
        if(curX == targetPoint.x && curY == targetPoint.y)
        {
            return CurNode; // Find a valid path
        }

        // Traversal the neighborhood
        for(int k = 0;k < neighbor.rows;k++)
        {
            int y = curY + neighbor.at<char>(k, 0);
            int x = curX + neighbor.at<char>(k, 1);
            if(x < 0 || x >= width || y < 0 || y >= height)
            {
                continue;
            }
            if(_LabelMap.at<uchar>(y, x) == free || _LabelMap.at<uchar>(y, x) == inOpenList)
            {
                // Determine whether a diagonal line can pass
                int dist1 = abs(neighbor.at<char>(k, 0)) + abs(neighbor.at<char>(k, 1));
                if(dist1 == 2 && _LabelMap.at<uchar>(y, curX) == obstacle && _LabelMap.at<uchar>(curY, x) == obstacle)
                    continue;

                // Calculate G, H, F value
                int addG, G, H, F;
                if(dist1 == 2)
                {
                    addG = 14;
                }
                else
                {
                    addG = 10;
                }
                G = CurNode->G + addG;
                if(Distance_ == 1)
                {   // Euclidean distance
                    int dist2 = (x - targetPoint.x) * (x - targetPoint.x) + (y - targetPoint.y) * (y - targetPoint.y);
                    H = round(10 * sqrt(dist2));
                }
                else if (Distance_ == 2)
                {   // Manhattan distance
                    H = 10 * (abs(x - targetPoint.x) + abs(y - targetPoint.y));
                }
                else if (Distance_ == 3)
                {   // Chebyshev distance
                    H = 10 * (std::max(abs(x - targetPoint.x), abs(y - targetPoint.y)));
                }
                else if (Distance_ == 4)
                {   // Diagonal distance
                    H = 10 * (abs(x - targetPoint.x) + abs(y - targetPoint.y) + (sqrt(2) - 2) * std::min(abs(x - targetPoint.x), abs(y - targetPoint.y)));
                }

                F = G + H;

                // Update the G, H, F value of node
                if(_LabelMap.at<uchar>(y, x) == free)
                {
                    Node* node = new Node();
                    node->point = cv::Point(x, y);
                    node->parent = CurNode;
                    node->G = G;
                    node->H = H;
                    node->F = F;
                    OpenList.push(std::pair<int, cv::Point>(node->F, node->point));
                    int index = point2index(node->point);
                    OpenDict[index] = node;
                    _LabelMap.at<uchar>(y, x) = inOpenList;
                }
                else // _LabelMap.at<uchar>(y, x) == inOpenList
                {
                    // Find the node
                    int index = point2index(cv::Point(x, y));
                    Node* node = OpenDict[index];
                    if(G < node->G)
                    {
                        node->G = G;
                        node->F = F;
                        node->parent = CurNode;
                    }
                }
            }
        }
    }

    return NULL; // Can not find a valid path
}

void AStarPlanner::GetPath(Node* TailNode, std::vector<cv::Point>& path)
{
    PathList.clear();
    path.clear();

    // Save path to PathList
    Node* CurNode = TailNode;
    while(CurNode != NULL)
    {
        PathList.push_back(CurNode);
        CurNode = CurNode->parent;
    }

    // Save path to std::vector<Point>
    int length = PathList.size();
    for(int i = 0;i < length;i++)
    {
        path.push_back(PathList.back()->point);
        PathList.pop_back();
    }

    // Release memory
    while(OpenList.size()) {
        cv::Point CurPoint = OpenList.top().second;
        OpenList.pop();
        int index = point2index(CurPoint);
        Node* CurNode = OpenDict[index];
        delete CurNode;
    }
    OpenDict.clear();
}