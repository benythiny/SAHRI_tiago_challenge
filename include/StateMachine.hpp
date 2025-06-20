#pragma once

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include "RobotTaskStatus.hpp"

#include "Motionplanning_arms.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

enum class RobotState
{
    IDLE,
    ADD_OBSTACLE,
    PICK_BLOCK,
    MOVE_TO_TABLE,
    PLACE_BLOCK,
    CHECK_LAYER,
    FINISHED
};

class StateMachine
{
public:
    StateMachine(rclcpp::Node::SharedPtr node);
    void update();

private:
    // params
    static constexpr double TABLE_X = 0.7;
    static constexpr double TABLE_Y = 0.0;
    static constexpr double TABLE_Z = 0.5;
    static constexpr double TABLE_DX = 0.2;
    static constexpr double TABLE_DY = 0.2;
    static constexpr double TABLE_DZ = 0.1;

    static constexpr double BLOCK_DZ = 0.015;
    static constexpr double BLOCK_DY = 0.025;
    static constexpr double BLOCK_DX = 0.075;

    static constexpr double GRIPPER_OFFSET = 0.2; // Distance from wrist to gripping point

    rclcpp::Node::SharedPtr node_;
    RobotState state_;
    int block_count_;
    int layer_count_;
    bool table_added_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;

    std::shared_ptr<icr_Motionplanning_arms> motion_node_;

    // Define Pick Pose
    geometry_msgs::msg::Pose pick_pose_;
    geometry_msgs::msg::Pose table_pose_;

    // add this member to hold the “next block” pose
    geometry_msgs::msg::PoseStamped next_block_pose_;

    // state actions
    void pickBlock();
    void addObstacle(geometry_msgs::msg::PoseStamped pose,
                     shape_msgs::msg::SolidPrimitive dimension);
    void moveToTable(const geometry_msgs::msg::PoseStamped &target);
    void placeBlock();
};
