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

// '.`\ /`|     _.-"```   `'.
// ; |  /   .'             `} 
// _\|\/_.-'                 }
// _.-"a                 {        }
// .-`  __    /._          {         }\
// '--"`  `""`   `\   ;    {         } \
//         |   } __ _\       }\  \
//         |  /;`   / :.   }`  \  \
//         | | | .-' /  / /     '. '._
// jgs    .'__/-' ````.-'.'        '-._'-._
//       ```        ````              `"""`

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create a shared pointer to your node
  auto node = std::make_shared<icr_Motionplanning_arms>();

  node->GripperControl("OPEN");
  // You can spin in a separate thread or just call your function directly
  try {
    double lift_value = 0.0;   // Example lift value within limits
    // node->TorsoControl(lift_value);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in TorsoControl: %s ", e.what());
  }
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;
  planning_scene_publisher_ = node->create_publisher<moveit_msgs::msg::PlanningScene>(
    "/planning_scene", rclcpp::QoS(1));

// Add obj table in pose 0.4 0.0 0.5
  geometry_msgs::msg::PoseStamped obstacle_pose;
  obstacle_pose.header.frame_id = "base_footprint";
  obstacle_pose.pose.position.x = 0.4;
  obstacle_pose.pose.position.y = 0.0;
  obstacle_pose.pose.position.z = 0.5;
  obstacle_pose.pose.orientation.x = 0.0;
  obstacle_pose.pose.orientation.y = 0.0;
  obstacle_pose.pose.orientation.z = 0.0;
  obstacle_pose.pose.orientation.w = 1.0;
  moveit_msgs::msg::PlanningScene planning_scene_msg =
    node->Add_Obstacle(obstacle_pose, "Table");
  //planning_scene_publisher_->publish(planning_scene_msg);


  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 0.4;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.5;

  // Simple orientation (quaternion), facing forward
  target_pose.orientation.x = 0.0;
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 1.0;

  try {
    node->motion_planning_control(target_pose, RobotTaskStatus::Arm::ARM_torso);  // or "arm" depending on your group name
    RCLCPP_INFO(node->get_logger(), "Motion planning succeeded.");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Motion planning failed: %s", e.what());
  }


  node->GripperControl("CLOSE");

  rclcpp::shutdown();
  return 0;
}
