#pragma once

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include "RobotTaskStatus.hpp"

enum class RobotState {
    IDLE,
    ADD_OBSTACLE,
    PICK_BLOCK,
    MOVE_TO_TABLE,
    PLACE_BLOCK,
    CHECK_LAYER,
    FINISHED
};

class StateMachine {
public:
    StateMachine(rclcpp::Node::SharedPtr node);

    void update();

private:
    rclcpp::Node::SharedPtr node_;
    RobotState state_;
    int block_count_;
    int layer_count_;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_;

    void pickBlock();
    void addObstacle(geometry_msgs::msg::PoseStamped pose, shape_msgs::msg::SolidPrimitive dimension);
    void moveToTable(int block_index, int layer_index);
    void placeBlock();
};
