
#pragma once

#include "PathPlannerStrategy.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <queue>
#include <unordered_map>


class AStarPlanner final : public PathPlannerStrategy
{
public:
    explicit AStarPlanner(const int distance = 1,
                    const int OccupyThresh = 50);
    ~AStarPlanner() override;

    nav_msgs::msg::Path plan(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal) override;

private:
    enum NodeType{
        obstacle = 0,
        free,
        inOpenList,
        inCloseList
    };

    struct Node{
        cv::Point point;    // node coordinate
        int F, G, H;        // cost
        Node* parent;       // parent node

        explicit Node(const cv::Point& _point = cv::Point(0, 0)):point(_point), F(0), G(0), H(0), parent(nullptr)
        {
        }
    };

    struct NodeCompare
    {
        bool operator() (const std::pair<int, cv::Point>& a, const std::pair<int, cv::Point>& b) const
        // Comparison function for priority queue
        {
            return a.first > b.first; // min heap
        }
    };

    static constexpr int COST_STRAIGHT = 10;
    static constexpr int COST_DIAGONAL = 14;

    void initAstar(const cv::Mat& map);
    void processMap();
    Node* findPath();
    static void reconstructPath(const Node* end_node, std::vector<cv::Point>& path);
    void cleanup();

    inline int pointToIndex(const cv::Point& point) const {
        return point.y * map_.cols + point.x;
    }

    inline cv::Point indexToPoint(const int index) const {
        // return cv::Point(static_cast<int>(index / map_.cols), index % map_.cols);
        return cv::Point(index % map_.cols, index / map_.cols);
    }

    inline bool isValid(const cv::Point& pt) const {
        return pt.x >= 0 && pt.x < map_.cols && pt.y >= 0 && pt.y < map_.rows;
    }

    inline bool isTraversable(const cv::Point& pt) const {
        return label_map_.at<uchar>(pt.y, pt.x) != obstacle;
    }

    int calculateHeuristic(const cv::Point& from, const cv::Point& to) const;

    //Object
    cv::Mat map_;
    cv::Mat label_map_;         // 标签地图
    cv::Point start_point_;     // 起点
    cv::Point target_point_;    // 目标点
    
    int distance_type_;         // 1:Euclidean  2:Manhattan  3:Chebyshev  4:Diagonal
    int occupy_thresh_;         // 0~255
    int inflate_radius_;        // integer

    std::priority_queue<std::pair<int, cv::Point>,
                        std::vector<std::pair<int, cv::Point>>,
                        NodeCompare> open_list_;      // open list
    std::unordered_map<int, Node*> open_dict_;        // open dict
    std::vector<Node*> all_nodes_;                    // path list
};

