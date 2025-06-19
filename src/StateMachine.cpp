#include "StateMachine.hpp"

StateMachine::StateMachine(rclcpp::Node::SharedPtr node)
: node_(node), state_(RobotState::IDLE), block_count_(0), layer_count_(0) { 

    // Create a shared pointer to your node
    motion_node_ = std::make_shared<icr_Motionplanning_arms>();

    // Publisher
    // planning_scene_publisher_ = motion_node->create_publisher<moveit_msgs::msg::PlanningScene>(
    // "/planning_scene", rclcpp::QoS(1));

    pick_pose_.position.x = 0.4;
    pick_pose_.position.y = 0.0;
    pick_pose_.position.z = 0.5;
    // Simple orientation (quaternion), facing forward
    pick_pose_.orientation.x = 0.0;
    pick_pose_.orientation.y = 0.707;
    pick_pose_.orientation.z = 0.0;
    pick_pose_.orientation.w = 0.707;


}


void StateMachine::update() {
    switch (state_) {
        case RobotState::IDLE:
            RCLCPP_INFO(node_->get_logger(), "Starting stacking...");
            state_ = RobotState::ADD_OBSTACLE;
            break;

        case RobotState::ADD_OBSTACLE:
            RCLCPP_INFO(node_->get_logger(), "Adding obstacle...");
            state_ = RobotState::PICK_BLOCK;
            break;

        case RobotState::PICK_BLOCK:
            RCLCPP_INFO(node_->get_logger(), "Picking block...");
            pickBlock();
            state_ = RobotState::MOVE_TO_TABLE;
            break;

        case RobotState::MOVE_TO_TABLE:
            RCLCPP_INFO(node_->get_logger(), "Moving to table...");
            moveToTable(block_count_, layer_count_);
            state_ = RobotState::PLACE_BLOCK;
            break;

        case RobotState::PLACE_BLOCK:
            RCLCPP_INFO(node_->get_logger(), "Placing block...");
            placeBlock();
            block_count_++;
            state_ = RobotState::CHECK_LAYER;
            break;

        case RobotState::CHECK_LAYER:
            if (block_count_ % 2 == 0) {
                layer_count_++;
            }
            state_ = RobotState::ADD_OBSTACLE;
            break;
    }
}

void StateMachine::pickBlock() {
    
    try {
        motion_node_->motion_planning_control(pick_pose_, RobotTaskStatus::Arm::ARM_torso);
        RCLCPP_INFO(node_->get_logger(), "Arm moved to pick position.");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to move to pick position: %s", e.what());
    }

    motion_node_->GripperControl("CLOSE");

}

void StateMachine::moveToTable(int block_index, int layer_index) {
    
    // Publish goal trajectory
    

}

void StateMachine::placeBlock() {
    
}
