#pragma once

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include "RobotTaskStatus.hpp"

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
    static constexpr double TABLE_X = 1.0; 
    static constexpr double TABLE_Y = 0.5; 
    static constexpr double TABLE_Z = 0.5;
    static constexpr double TABLE_DX = 1.2; 
    static constexpr double TABLE_DY = 1.2; 
    static constexpr double TABLE_DZ = 0.1;

    static constexpr double BLOCK_DX = 0.15;
    static constexpr double BLOCK_DY = 0.025;
    static constexpr double BLOCK_DZ = 0.05;

    rclcpp::Node::SharedPtr node_;
    RobotState state_;
    int block_count_;
    int layer_count_;
    bool table_added_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;

    // state actions
    void pickBlock();
    void addObstacle(geometry_msgs::msg::PoseStamped pose,
                     shape_msgs::msg::SolidPrimitive dimension);
    void moveToTable(int block_index, int layer_index);
    void placeBlock();
};
