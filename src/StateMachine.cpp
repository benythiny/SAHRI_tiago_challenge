#include "StateMachine.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>

StateMachine::StateMachine(rclcpp::Node::SharedPtr node)
    : node_(node),
      state_(RobotState::IDLE),
      block_count_(0),
      layer_count_(0),
      table_added_(false)
{
    // Create a shared pointer to your node
    motion_node_ = std::make_shared<icr_Motionplanning_arms>();

    // Publisher
    planning_scene_pub_ = motion_node_->create_publisher<moveit_msgs::msg::PlanningScene>(
        "/planning_scene", rclcpp::QoS(1));

    pick_pose_.position.x = 0.7;
    pick_pose_.position.y = 0.0;
    pick_pose_.position.z = 1.0;
    // Simple orientation (quaternion), facing forward
    pick_pose_.orientation.x = 0.0;
    pick_pose_.orientation.y = 0.707;
    pick_pose_.orientation.z = 0.0;
    pick_pose_.orientation.w = 0.707;

    table_pose_.position.x = 0.8;
    table_pose_.position.y = 0.0;
    table_pose_.position.z = 1.0;
    // Simple orientation (quaternion), facing forward
    table_pose_.orientation.x = 0.0;
    table_pose_.orientation.y = 0.707;
    table_pose_.orientation.z = 0.0;
    table_pose_.orientation.w = 0.707;

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

        // common header
        pose.header.frame_id = "base_footprint";
        pose.header.stamp = node_->now();

        if (!table_added_)
        {
            // 1) Add the table with identity orientation
            pose.pose.position.x = TABLE_X;
            pose.pose.position.y = TABLE_Y;
            pose.pose.position.z = TABLE_Z;
            pose.pose.orientation.w = 1.0;

            dim.type = dim.BOX;
            dim.dimensions = {TABLE_DX, TABLE_DY, TABLE_DZ};

            table_added_ = true;
        }
        else
        {
            // 2) Compute block pose & orientation
            // 2a) layer orientation: even layers = Y‐axis blocks (90°), odd = X‐axis (0°)
            bool even_layer = (layer_count_ % 2 == 0);
            double yaw = even_layer ? M_PI_2 : 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            pose.pose.orientation = tf2::toMsg(q);

            // 2b) Z so block sits centered on its layer
            double table_top = TABLE_Z + TABLE_DZ / 2.0;
            double z = table_top + layer_count_ * BLOCK_DZ // full layers below
                       + BLOCK_DZ / 2.0;                   // center of this block
            pose.pose.position.z = z;

            // 2c) X/Y offset: two blocks per layer, spaced BLOCK_DY apart
            double sep = BLOCK_DY;
            bool first_in_layer = (block_count_ % 2 == 0);
            double side = first_in_layer ? -sep : sep;

            if (even_layer)
            {
                // blocks long‐axis = Y ⇒ offset in X
                pose.pose.position.x = TABLE_X + side;
                pose.pose.position.y = TABLE_Y;
            }
            else
            {
                // blocks long‐axis = X ⇒ offset in Y
                pose.pose.position.x = TABLE_X;
                pose.pose.position.y = TABLE_Y + side;
            }

            dim.type = dim.BOX;
            dim.dimensions = {BLOCK_DX, BLOCK_DY, BLOCK_DZ};
            block_count_++;
            // save it for MOVE_TO_TABLE
            next_block_pose_ = pose;

            // DEBUG: print out what you just computed
            RCLCPP_INFO(node_->get_logger(),
                        "[ADD_OBSTACLE] next_block_pose_ → frame: %s, x=%.3f y=%.3f z=%.3f",
                        next_block_pose_.header.frame_id.c_str(),
                        next_block_pose_.pose.position.x,
                        next_block_pose_.pose.position.y,
                        next_block_pose_.pose.position.z);
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
        RCLCPP_INFO(node_->get_logger(), "[MOVE_TO_TABLE] moving to:");
        RCLCPP_INFO(node_->get_logger(),
                    "   frame: %s  x=%.3f y=%.3f z=%.3f",
                    next_block_pose_.header.frame_id.c_str(),
                    next_block_pose_.pose.position.x,
                    next_block_pose_.pose.position.y,
                    next_block_pose_.pose.position.z);
        moveToTable(next_block_pose_);
        state_ = RobotState::PLACE_BLOCK;
        break;

    case RobotState::PLACE_BLOCK:
        RCLCPP_INFO(node_->get_logger(), "Placing block…");
        placeBlock();
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

//
void StateMachine::pickBlock()
{
    bool success = false;
    auto start = std::chrono::steady_clock::now();
    const auto TIMEOUT = std::chrono::seconds(5);

    while (!success)
    {
        try
        {
            motion_node_->motion_planning_control(
                pick_pose_, RobotTaskStatus::Arm::ARM_torso);
            RCLCPP_INFO(node_->get_logger(), "Arm moved to pick position.");
            success = true;
        }
        catch (const std::exception &e)
        {
            auto elapsed = std::chrono::steady_clock::now() - start;
            if (elapsed > TIMEOUT)
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "Pick planning timed out after %lds",
                             TIMEOUT.count());
                break;
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(),
                            "Pick planning failed (%.2fs elapsed), retrying: %s",
                            std::chrono::duration<double>(elapsed).count(),
                            e.what());
            }
        }
    }

    if (success)
    {
        motion_node_->GripperControl("CLOSE");
    }
}

void StateMachine::moveToTable(
    const geometry_msgs::msg::PoseStamped &target)
{
    // apply gripper offset
    geometry_msgs::msg::PoseStamped wrist_target = target;
    wrist_target.pose.position.z += GRIPPER_OFFSET;

    bool success = false;
    auto start = std::chrono::steady_clock::now();
    const auto TIMEOUT = std::chrono::seconds(5);

    while (!success)
    {
        try
        {
            motion_node_->motion_planning_control(
                wrist_target.pose, RobotTaskStatus::Arm::ARM_torso);
            RCLCPP_INFO(node_->get_logger(),
                        "Arm moved to block pose (wrist offset by %.3fm).",
                        GRIPPER_OFFSET);
            success = true;
        }
        catch (const std::exception &e)
        {
            auto elapsed = std::chrono::steady_clock::now() - start;
            if (elapsed > TIMEOUT)
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "Move-to-table planning timed out after %lds",
                             TIMEOUT.count());
                break;
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(),
                            "Move-to-table planning failed (%.2fs elapsed), retrying: %s",
                            std::chrono::duration<double>(elapsed).count(),
                            e.what());
            }
        }
    }
}

void StateMachine::placeBlock()
{
    // Log what we’re doing
    RCLCPP_INFO(node_->get_logger(),
                "Placing block %d in layer %d",
                block_count_, layer_count_);

    // Build a PoseStamped for the place location
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_footprint"; // <-- add semicolon
    target_pose.header.stamp = node_->now();

    // 1) compute Z so the block sits on top of the correct layer
    double table_top_z = TABLE_Z + TABLE_DZ / 2.0;
    double z = table_top_z + layer_count_ * BLOCK_DZ // full layers below
               + BLOCK_DZ / 2.0;                     // center of this block
    target_pose.pose.position.z = z;

    // 2) pick orientation: even layers = 90°, odd = 0°
    bool even_layer = (layer_count_ % 2 == 0);
    double yaw = even_layer ? M_PI_2 : 0.0; // use the M_PI_2 macro from <cmath>
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    target_pose.pose.orientation = tf2::toMsg(q);

    // 3) for each layer there are 2 blocks: index 0 or 1 in that layer
    //    we space them one block‐width (BLOCK_DY) apart along the axis
    double sep = BLOCK_DY; // gap = block “width”
    bool first_in_layer = (block_count_ % 2 == 0);
    double side = first_in_layer ? -sep : sep;

    if (even_layer)
    {
        // blocks long‐axis = Y, so separate along X
        target_pose.pose.position.x = TABLE_X + side;
        target_pose.pose.position.y = TABLE_Y;
    }
    else
    {
        // blocks long‐axis = X, so separate along Y
        target_pose.pose.position.x = TABLE_X;
        target_pose.pose.position.y = TABLE_Y + side;
    }

    // DEBUG print
    RCLCPP_INFO(node_->get_logger(),
                " -> [x=%.3f y=%.3f z=%.3f yaw=%.2f°]",
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z,
                yaw * 180.0 / M_PI);

    {
        // before dropping, hold back by the gripper offset
        geometry_msgs::msg::PoseStamped wrist_place = target_pose;
        wrist_place.pose.position.z += GRIPPER_OFFSET;
        bool success = false;
        auto start = std::chrono::steady_clock::now();
        const auto TIMEOUT = std::chrono::seconds(5);

        // retry loop
        while (!success)
        {
            try
            {
                motion_node_->motion_planning_control(
                    wrist_place.pose, RobotTaskStatus::Arm::ARM_torso);
                RCLCPP_INFO(node_->get_logger(),
                            "Arm moved to place pose (wrist offset by %.3fm).",
                            GRIPPER_OFFSET);
                success = true;
            }
            catch (const std::exception &e)
            {
                auto elapsed = std::chrono::steady_clock::now() - start;
                if (elapsed > TIMEOUT)
                {
                    RCLCPP_ERROR(node_->get_logger(),
                                 "Place planning timed out after %lds",
                                 TIMEOUT.count());
                    break;
                }
                else
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "Place planning failed (%.2fs elapsed), retrying: %s",
                                std::chrono::duration<double>(elapsed).count(),
                                e.what());
                }
            }
        }

        if (success)
        {
            motion_node_->GripperControl("OPEN");
        }
    }
}
