#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include "geometry_msgs/Pose.h"
#include "std_msgs/Int32.h"
#include "tf/transform_datatypes.h"

class ControlCenter{
public:
    ControlCenter(){
      gripper_status.data = 0;
      gripper_pub = node_handle.advertise<std_msgs::Int32>("/gripper_control/int32", 1);
    }

    std_msgs::Int32 gripper_status;//0-打开 1-关闭
    ros::Publisher gripper_pub;
private:
    ros::NodeHandle node_handle;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_control");
  ControlCenter center;

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
    move_group.setJointValueTarget(joint_home_positions);
    move_group.setMaxVelocityScalingFactor(1);
    ROS_INFO("Go to home");
    move_group.move();

    center.gripper_status.data = 0;
    center.gripper_pub.publish(center.gripper_status);

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to pick");
    // Planning to a pick
    // ^^^^^^^^^^^^^^^^^^^^^^^
    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = 0.008;
    target_pose.orientation.y = -0.721;
    target_pose.orientation.z = 0.008;
    target_pose.orientation.w = 0.692;
    target_pose.position.x = -0.33;
    target_pose.position.y = 0;
    target_pose.position.z = 0.7;

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose sub_pose;
    sub_pose = target_pose;
    sub_pose.position.x += 0.1;
    waypoints.push_back(sub_pose);

    waypoints.push_back(target_pose);

    move_group.setMaxVelocityScalingFactor(0.02);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.02;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    
    if (fraction>0.5)
      move_group.execute(trajectory);
      sleep(10);
      center.gripper_status.data = 1;
      center.gripper_pub.publish(center.gripper_status);

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to pacle");
    // Planning to a place
    // ^^^^^^^^^^^^^^^^^^^^^^^
    std::vector<geometry_msgs::Pose> place_waypoints;
    geometry_msgs::Pose place_pose;
    place_pose = target_pose;

    place_pose.position.x += 0.1;
    place_waypoints.push_back(place_pose);
    place_pose.position.x += 0.1;
    place_waypoints.push_back(place_pose);

    move_group.setMaxVelocityScalingFactor(0.2);
    moveit_msgs::RobotTrajectory place_trajectory;
    double place_fraction = move_group.computeCartesianPath(place_waypoints, eef_step, jump_threshold, place_trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing (Cartesian path) (%.2f%% acheived)", place_fraction * 100.0);
    
    if (place_fraction>0.5)
      move_group.execute(place_trajectory);
      sleep(5);
      center.gripper_status.data = 0;
      center.gripper_pub.publish(center.gripper_status);
    
    // 3333333
    // ^^^^^^^^^^^^^^^^^^^^^^^
    std::vector<geometry_msgs::Pose> waypoints_3;

    place_pose.position.x += 0.1;
    waypoints_3.push_back(place_pose);

    move_group.setMaxVelocityScalingFactor(0.2);
    moveit_msgs::RobotTrajectory trajectory_3;
    double fraction_3 = move_group.computeCartesianPath(waypoints_3, eef_step, jump_threshold, trajectory_3);
    ROS_INFO_NAMED("tutorial", "Visualizing (Cartesian path) (%.2f%% acheived)", fraction_3 * 100.0);
    
    if (fraction_3>0.5)
      move_group.execute(trajectory_3);
  }
  ros::shutdown();
  return 0;
}
