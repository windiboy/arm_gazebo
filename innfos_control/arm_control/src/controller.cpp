#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_control");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // init
  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  
  // visual
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("dummy");
  visual_tools.deleteAllMarkers();

  // print info
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
  
  // work flow
  while(ros::ok()){
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to go to home");
    std::vector<double> joint_home_positions(6, 0.0);
    std::vector<double> joint_values(6, 0.0);
    joint_values[0] = 1.504099;
    joint_values[1] = 1.021270;
    joint_values[2] = -1.022914;
    joint_values[3] = -1.027853;
    joint_values[4] = -0.735521;
    joint_values[5] = 2.446306;
    move_group.setJointValueTarget(joint_values);
    ROS_INFO("Go to home");
    move_group.move();

    // Start the demo
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // Planning to a Pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.x = 0.008;
    target_pose1.orientation.y = -0.721;
    target_pose1.orientation.z = 0.008;
    target_pose1.orientation.w = 0.692;
    target_pose1.position.x = -0.315;
    target_pose1.position.y = 0;
    target_pose1.position.z = 0.634;
    move_group.setPoseTarget(target_pose1);

    // joint space
    // moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    // std::vector<double> joint_group_positions;
    // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    // joint_group_positions[4] = 1.0;  // radians
    // move_group.setJointValueTarget(joint_group_positions);

    // plan and excute
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = false;
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    if (success)
      move_group.execute(my_plan.trajectory_);
  }
  ros::shutdown();
  return 0;
}
