#include <rclcpp/rclcpp.hpp>
#include "StateMachine.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("brick_stacker_node");

    StateMachine machine(node);

    RCLCPP_INFO(node->get_logger(), "Hello");

    // auto timer = node->create_wall_timer(
    //     std::chrono::milliseconds(500),
    // [&machine]() { machine.update(); });
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
