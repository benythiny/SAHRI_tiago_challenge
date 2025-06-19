#pragma once

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include "RobotTaskStatus.hpp"

#include "Motionplanning_arms.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

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

    // Publishers
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;

    std::shared_ptr<icr_Motionplanning_arms> motion_node_;

    // Define Pick Pose
    geometry_msgs::msg::Pose pick_pose_;

    void pickBlock();
    void addObstacle(geometry_msgs::msg::PoseStamped pose, shape_msgs::msg::SolidPrimitive dimension);
    void moveToTable(int block_index, int layer_index);
    void placeBlock();
};
