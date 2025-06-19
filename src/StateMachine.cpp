#include "StateMachine.hpp"

StateMachine::StateMachine(rclcpp::Node::SharedPtr node)
    : node_(node),
      state_(RobotState::IDLE),
      block_count_(0),
      layer_count_(0),
      table_added_(false)
{
    planning_scene_pub_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>(
        "/monitored_planning_scene", rclcpp::QoS(1));
    RCLCPP_INFO(node_->get_logger(), "StateMachine up");
}

void StateMachine::update()
{
    switch (state_)
    {
    case RobotState::IDLE:
        RCLCPP_INFO(node_->get_logger(), "Starting stacking...");
        state_ = RobotState::ADD_OBSTACLE;
        break;

    case RobotState::ADD_OBSTACLE:
    {
        RCLCPP_INFO(node_->get_logger(), "Adding obstacle…");
        geometry_msgs::msg::PoseStamped pose;
        shape_msgs::msg::SolidPrimitive dim;

        pose.header.frame_id = "base_footprint";
        pose.pose.position.x = TABLE_X;
        pose.pose.position.y = TABLE_Y;
        pose.pose.orientation.w = 1.0;

        if (!table_added_)
        {
            // first time: add table
            pose.pose.position.z = TABLE_Z;
            dim.type = dim.BOX;
            dim.dimensions = {TABLE_DX, TABLE_DY, TABLE_DZ};
            table_added_ = true;
        }
        else
        {
            // every subsequent call: stack a Jenga block
            double table_top_z = TABLE_Z - TABLE_DZ / 2.0;
            double z = table_top_z + block_count_ * BLOCK_DZ // full layers below
                       + BLOCK_DZ / 2.0;                               // half above current layer
            pose.pose.position.z = z;
            dim.type = dim.BOX;
            dim.dimensions = {BLOCK_DX, BLOCK_DY, BLOCK_DZ};
        }

        addObstacle(pose, dim);
        state_ = RobotState::PICK_BLOCK;
        break;
    }

    case RobotState::PICK_BLOCK:
        RCLCPP_INFO(node_->get_logger(), "Picking block…");
        pickBlock();
        state_ = RobotState::MOVE_TO_TABLE;
        break;

    case RobotState::MOVE_TO_TABLE:
        RCLCPP_INFO(node_->get_logger(), "Moving to table…");
        moveToTable(block_count_, layer_count_);
        state_ = RobotState::PLACE_BLOCK;
        break;

    case RobotState::PLACE_BLOCK:
        RCLCPP_INFO(node_->get_logger(), "Placing block…");
        placeBlock();
        block_count_++;
        state_ = RobotState::CHECK_LAYER;
        break;

    case RobotState::CHECK_LAYER:
        if (block_count_ % 2 == 0)
        {
            layer_count_++;
        }
        state_ = RobotState::ADD_OBSTACLE;
        break;

    default:
        break;
    }
}

void StateMachine::addObstacle(geometry_msgs::msg::PoseStamped pose,
                               shape_msgs::msg::SolidPrimitive dim)
{
    moveit_msgs::msg::CollisionObject co;
    co.header.frame_id = pose.header.frame_id;
    co.id = table_added_ ? "block_" + std::to_string(block_count_) : "table";
    co.primitives.push_back(dim);
    co.primitive_poses.push_back(pose.pose);
    co.operation = moveit_msgs::msg::CollisionObject::ADD;

    moveit_msgs::msg::PlanningScene scene;
    scene.world.collision_objects.push_back(co);
    scene.is_diff = true;
    planning_scene_pub_->publish(scene);
}

// stubs…
void StateMachine::pickBlock() {}
void StateMachine::moveToTable(int, int) {}
void StateMachine::placeBlock() {}
