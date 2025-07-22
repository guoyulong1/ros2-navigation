#pragma once

#include "robot_navigation/PathPlannerStrategy.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <opencv2/opencv.hpp>


class AStarPlanner : public PathPlannerStrategy
{
public:
    AStarPlanner(const int distance, const int OccupyThresh, const int InflateRadius);
    nav_msgs::msg::Path plan(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal) override;

    void InitAstar(cv::Mat& _Map);

public:
    enum NodeType{
        obstacle = 0,
        free,
        inOpenList,
        inCloseList
    };

    struct Node{
        cv::Point point;  // node coordinate
        int F, G, H;  // cost
        Node* parent;  // parent node

        Node(cv::Point _point = cv::Point(0, 0)):point(_point), F(0), G(0), H(0), parent(NULL)
        {
        }
    };

    struct cmp
    {
        bool operator() (std::pair<int, cv::Point> a, std::pair<int, cv::Point> b) // Comparison function for priority queue
        {
            return a.first > b.first; // min heap
        }
    };


public:
    inline int point2index(cv::Point point) {
        return point.y * Map.cols + point.x;
    }
    inline cv::Point index2point(int index) {
        return cv::Point(int(index / Map.cols), index % Map.cols);
    }

    inline bool isValid(const cv::Point& pt) const {
        return pt.x >= 0 && pt.x < Map.cols && pt.y >= 0 && pt.y < Map.rows;
    }

private:
    void MapProcess(cv::Mat& Mask);
    Node* FindPath();
    void GetPath(Node* TailNode, std::vector<cv::Point>& path);

private:
    //Object
    cv::Mat Map;
    cv::Point startPoint, targetPoint;
    cv::Mat neighbor;
    cv::Mat LabelMap;
    
    int Distance_;       // 1:Euclidean  2:Manhattan  3:Chebyshev  4:Diagonal 
    int OccupyThresh_;   // 0~255
    int InflateRadius_;  // integer

    std::priority_queue<std::pair<int, cv::Point>, std::vector<std::pair<int, cv::Point>>, cmp> OpenList; // open list
    std::unordered_map<int, Node*> OpenDict; // open dict
    std::vector<Node*> PathList;  // path list
};

