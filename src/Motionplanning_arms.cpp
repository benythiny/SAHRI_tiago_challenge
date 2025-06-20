// ROS2 equivalent of icr_Motionplanning_arms.cpp (completed conversion)

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "std_msgs/msg/string.hpp"

#include "Motionplanning_arms.hpp"
#include "RobotTaskStatus.hpp"

#include <rclcpp_action/rclcpp_action.hpp>
#include <play_motion2_msgs/action/play_motion2.hpp>
#include <control_msgs/action/point_head.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <yaml-cpp/yaml.h>
#include <memory>
#include <chrono>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using GoalHandleFollowJointTrajectory =
  rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>;


void icr_Motionplanning_arms::TorsoControl(double lift_value)
{
  RCLCPP_INFO(this->get_logger(), "Creating action client to torso controller ...");

  torso_control_client_ptr action_client =
    rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
    this, "/torso_controller/follow_joint_trajectory");

  // Wait for action server with timeout loop
  int iterations = 0;
  const int max_iterations = 3;
  const auto wait_time = std::chrono::seconds(2);

  while (!action_client->wait_for_action_server(wait_time) && rclcpp::ok() &&
    iterations < max_iterations)
  {
    RCLCPP_DEBUG(this->get_logger(), "Waiting for the torso server to come up");
    ++iterations;
  }

  if (iterations == max_iterations) {
    throw std::runtime_error(
            "Error in create torso client: torso controller action server not available");
  }

  // Call your action sending method here (you need to implement TorsoSendAction accordingly)
  TorsoSendAction(action_client, lift_value);
}


void icr_Motionplanning_arms::TorsoSendAction(
  torso_control_client_ptr torso_client,
  double lift_value)
{
  auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();

  RCLCPP_INFO(this->get_logger(), "torso joint");

  goal_msg.trajectory.joint_names.push_back("torso_lift_joint");
  goal_msg.trajectory.points.resize(1);
  goal_msg.trajectory.points[0].positions.resize(1);
  goal_msg.trajectory.points[0].positions[0] = lift_value;  // Max 0.35  Min 0.00

  goal_msg.trajectory.points[0].velocities.resize(1);
  goal_msg.trajectory.points[0].velocities[0] = 0.0;

  goal_msg.trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(2.0);

  // Send goal asynchronously and wait for result synchronously
  auto send_goal_options =
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();

  // Optional: feedback callback
  send_goal_options.feedback_callback =
    [](GoalHandleFollowJointTrajectory::SharedPtr,
      const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Feedback received");
      // Handle feedback if needed
    };

  auto goal_future = torso_client->async_send_goal(goal_msg, send_goal_options);

  // Wait for the goal to be accepted
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Send goal call failed");
    return;
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    return;
  }

  // Wait for the result
  auto result_future = torso_client->async_get_result(goal_handle);

  RCLCPP_INFO(this->get_logger(), "Waiting for result ...");

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to get result");
    return;
  }

  auto wrapped_result = result_future.get();

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Action Torso finished successfully");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Action Torso was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Action Torso was canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
  }
}

/*Gripper functions*/

void icr_Motionplanning_arms::GripperControl(const std::string & command)
{
  gripper_control_client_ptr gripper_client;
  const std::string gripper_controller_name = "gripper_controller";  // single generic controller name
  const std::vector<std::string> joint_names = {
    "gripper_left_finger_joint",  // or just "gripper_finger_joint", adapt as needed
    "gripper_right_finger_joint"
  };

  try {
    createGripperClient(gripper_client, gripper_controller_name);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create gripper client: %s", e.what());
    return;
  }

  SendGripperAction(gripper_client, joint_names, command);
}

void icr_Motionplanning_arms::createGripperClient(
  gripper_control_client_ptr & client,
  const std::string & controller_name)
{
  std::string action_name = "/" + controller_name + "/follow_joint_trajectory";
  client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
    this,
    action_name);

  if (!client->wait_for_action_server(std::chrono::seconds(5))) {
    throw std::runtime_error("Action server not available: " + action_name);
  }
}

void icr_Motionplanning_arms::SendGripperAction(
  gripper_control_client_ptr client,
  const std::vector<std::string> & joint_names,
  const std::string & command)
{
  using GoalHandleFollowJointTrajectory =
    rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>;

  control_msgs::action::FollowJointTrajectory::Goal goal;

  goal.trajectory.joint_names = joint_names;
  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].positions.resize(joint_names.size());

  if (command == "OPEN") {
    for (size_t i = 0; i < joint_names.size(); ++i) {
      goal.trajectory.points[0].positions[i] = 0.04;  // example open pos
    }
  } else if (command == "CLOSE") {
    for (size_t i = 0; i < joint_names.size(); ++i) {
      goal.trajectory.points[0].positions[i] = 0.01;  // example close pos
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown gripper command: %s", command.c_str());
    return;
  }

  goal.trajectory.points[0].velocities.resize(joint_names.size(), 0.0);
  goal.trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(1.0);

  // Send goal asynchronously and wait for result future
  auto send_goal_options =
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();

  send_goal_options.result_callback =
    [this](const GoalHandleFollowJointTrajectory::WrappedResult & result) {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Gripper action succeeded");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Gripper action aborted");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Gripper action canceled");
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          break;
      }
    };

  auto goal_handle_future = client->async_send_goal(goal, send_goal_options);
}

using PlayMotion2 = play_motion2_msgs::action::PlayMotion2;
using GoalHandlePlayMotion2 = rclcpp_action::ClientGoalHandle<PlayMotion2>;

void icr_Motionplanning_arms::ARM_Homeposition()
{
  // Create action client (assuming this class has a node pointer or inherits rclcpp::Node)
  auto client = rclcpp_action::create_client<PlayMotion2>(
    this, "play_motion");

  if (!client->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "play_motion action server not available after waiting");
    return;
  }

  auto goal_msg = PlayMotion2::Goal();
  goal_msg.motion_name = "home";

  RCLCPP_INFO(
    this->get_logger(), "Sending play_motion2 goal with motion: %s", goal_msg.motion_name.c_str());

  // Setup goal response callback and result callback
  auto send_goal_options = rclcpp_action::Client<PlayMotion2>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [](std::shared_ptr<GoalHandlePlayMotion2> goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal rejected by server");
      } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal accepted by server, executing...");
      }
    };


  send_goal_options.result_callback =
    [this](const GoalHandlePlayMotion2::WrappedResult & result) {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "play_motion2 succeeded");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "play_motion2 aborted");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(this->get_logger(), "play_motion2 canceled");
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          break;
      }
    };

  // Send the goal asynchronously
  client->async_send_goal(goal_msg, send_goal_options);

  // Optionally, you can spin until result or handle callbacks in your node's executor
}


int icr_Motionplanning_arms::ArmMotionPlanning(RobotTaskStatus action_to_do)
{

  // Single arm group, e.g. ARM_SINGLE
  if (RobotTaskStatus::armToString(action_to_do.getArm()) == "arm") {
    motion_planning_control(
      action_to_do.getGoal(),
      RobotTaskStatus::Arm::ARM);
  }
  // Arm with torso group, e.g. ARM_TORSO
  else if (RobotTaskStatus::armToString(action_to_do.getArm()) == "arm_torso") {
    motion_planning_control(
      action_to_do.getGoal(),
      RobotTaskStatus::Arm::ARM_torso);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Error, planning not well posed");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

void icr_Motionplanning_arms::motion_planning_control(
  const geometry_msgs::msg::Pose & goal,
  RobotTaskStatus::Arm motion_type)
{
  std::string moveit_group_selection;

  if (motion_type == RobotTaskStatus::Arm::ARM) {
    moveit_group_selection = "arm";  // adjust to your setup
  } else if (motion_type == RobotTaskStatus::Arm::ARM_torso) {
    moveit_group_selection = "arm_torso";  // adjust to your setup
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid motion type for planning");
    return;
  }

  const std::string end_effector_link = "arm_tool_link";   // adjust to your arm's tool link

  // Initialize MoveGroupInterface for the arm
  moveit::planning_interface::MoveGroupInterface group_arm(shared_from_this(),
    moveit_group_selection);

  // Set the target pose (assume base frame is "base_footprint")
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "base_footprint";
  target_pose.pose = goal;
  group_arm.setPoseTarget(target_pose, end_effector_link);

  // Set planner parameters
  group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setStartStateToCurrentState();
  group_arm.setMaxVelocityScalingFactor(1.0);
  group_arm.setNumPlanningAttempts(5);
  group_arm.setGoalTolerance(0.001);
  group_arm.allowReplanning(true);
  group_arm.setPoseReferenceFrame("base_footprint");
  group_arm.setPlanningTime(0.1);

  // Plan and execute
  moveit::planning_interface::MoveItErrorCode success = group_arm.move();

  if (!success) {
    throw std::runtime_error("Error executing motion planning");
  }
}

moveit_msgs::msg::PlanningScene icr_Motionplanning_arms::Add_Obstacle(
  const geometry_msgs::msg::PoseStamped & obstacle_pose,
  const std::string & obstacle_id)
{
  moveit_msgs::msg::CollisionObject collision_object;
  shape_msgs::msg::SolidPrimitive primitive;
  geometry_msgs::msg::Pose pose_collision_object;

  if (obstacle_id == "Table") {
    collision_object.header.frame_id = "base_footprint";
    collision_object.id = obstacle_id;

    primitive.type = primitive.BOX;
    primitive.dimensions = {0.4, 10.0, 0.1};  // x, y, z

    pose_collision_object.position.x = obstacle_pose.pose.position.x +
      (primitive.dimensions[0] / 2.0);
    pose_collision_object.position.y = obstacle_pose.pose.position.y;
    pose_collision_object.position.z = obstacle_pose.pose.position.z -
      (primitive.dimensions[2] / 2.0) + 0.002;

    pose_collision_object.orientation = obstacle_pose.pose.orientation;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose_collision_object);
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
  }

  moveit_msgs::msg::PlanningScene planning_scene_msg;
  planning_scene_msg.world.collision_objects.push_back(collision_object);
  planning_scene_msg.is_diff = true;

  return planning_scene_msg;
}
