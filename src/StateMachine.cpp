#include "StateMachine.hpp"

StateMachine::StateMachine(rclcpp::Node::SharedPtr node)
: node_(node), state_(RobotState::IDLE), block_count_(0), layer_count_(0) {

    RCLCPP_INFO(node_->get_logger(),"Hello2");

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
    
}

void StateMachine::moveToTable(int block_index, int layer_index) {
    
}

void StateMachine::placeBlock() {
    
}
