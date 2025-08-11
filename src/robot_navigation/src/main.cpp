#include <rclcpp/rclcpp.hpp>
#include "robot_navigation/navigation_node.hpp"
#include "log/logging.hpp"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    init_logger();
    auto node = std::make_shared<NavigationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}