#include <rclcpp/rclcpp.hpp>
#include "robot_navigation/navigation_node.hpp"
#include "log/logging.hpp"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    init_logger();
    auto Navigation_node = std::make_shared<NavigationNode>();
    rclcpp::spin(Navigation_node);
    rclcpp::shutdown();
    return 0;
}