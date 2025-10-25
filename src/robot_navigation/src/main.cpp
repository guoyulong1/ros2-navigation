#include <rclcpp/rclcpp.hpp>
#include "navigation_manager.hpp"
#include "utils/logger.hpp"
#include "comm_manager.hpp"

int main(const int argc, char **argv)
{
    rclcpp::init(argc, argv);
    init_logger();
    auto& comm_manager = CommManager::Instance();
    comm_manager.initialize();
    rclcpp::spin(comm_manager.get_node_base_interface());

    auto& navigation_node = NavigationNode::getInstance();
    rclcpp::spin(navigation_node.get_node_base_interface());

    rclcpp::shutdown();
    return 0;
}