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
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ControlCenter
{
public:
  ControlCenter() {}
  tf::TransformListener listener;
  tf::StampedTransform transform;
  ros::NodeHandle node_handle;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_control");
  ControlCenter center;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // init
  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface gripper_group("gripper");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // print info
  ROS_INFO_NAMED("[open_door]", "Planning frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("[open_door]", "End effector link: %s", move_group.getEndEffectorLink().c_str());
  ROS_INFO_NAMED("[open_door]", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
  std::cout << std::endl;

  // 从文件读取配置
  XmlRpc::XmlRpcValue target_pose_params;
  geometry_msgs::Pose target_pose;
  tf2::Quaternion orientation;
  center.node_handle.getParam("target_pose", target_pose_params);
  for (size_t i = 0; i < target_pose_params.size(); ++i)
  {
    orientation.setRPY(target_pose_params[i]["orientation_r"], target_pose_params[i]["orientation_p"], target_pose_params[i]["orientation_y"]);
    target_pose.orientation = tf2::toMsg(orientation);
    target_pose.position.x = target_pose_params[i]["pos_x"];
    target_pose.position.y = target_pose_params[i]["pos_y"];
    target_pose.position.z = target_pose_params[i]["pos_z"];
  }
  std::cout << "[open_door]" << target_pose << std::endl;

  // work flow
  while (ros::ok())
  {
    //round1
    std::vector<double> joint_home_positions(6, 0.0);
    move_group.setJointValueTarget(joint_home_positions);
    // move_group.setMaxVelocityScalingFactor(5);
    ROS_INFO("Go to home");
    gripper_group.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.0);
    gripper_group.move();
    move_group.move();

    //round2
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose sub_pose;
    sub_pose = target_pose;
    // sub_pose.position.x = 0.0;
    // sub_pose.position.y = 0.0;
    // sub_pose.position.z = 0.7;
    sub_pose.position.y -= 0.1;
    waypoints.push_back(sub_pose);
    waypoints.push_back(target_pose);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.02;
    double fraction = 0.0;
    while (fraction < 0.5)
    {
      fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      ROS_INFO_NAMED("tutorial", "Visualizing (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    }
    if (fraction > 0.5)
    {
      move_group.execute(trajectory);
      gripper_group.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.4);
      gripper_group.move();
    }
    else
      break;

    //round3
    std::vector<geometry_msgs::Pose> waypoints_3;
    geometry_msgs::Pose second_pose;
    second_pose = target_pose;
    // orientation.setRPY(M_PI * 3 / 4, 0, 0);
    // second_pose.orientation = tf2::toMsg(orientation);
    second_pose.position.z -= 0.05;
    waypoints_3.push_back(second_pose);
    moveit_msgs::RobotTrajectory trajectory_3;
    double fraction_3 = move_group.computeCartesianPath(waypoints_3, eef_step, jump_threshold, trajectory_3);
    ROS_INFO_NAMED("tutorial", "Visualizing (Cartesian path) (%.2f%% acheived)", fraction_3 * 100.0);
    if (fraction_3 > 0.5)
      move_group.execute(trajectory_3);
    else
      break;

    //round4
    std::vector<geometry_msgs::Pose> place_waypoints;
    geometry_msgs::Pose place_pose;
    place_pose = second_pose;
    place_pose.position.y -= 0.04;
    place_waypoints.push_back(place_pose);
    place_pose.position.y -= 0.04;
    place_waypoints.push_back(place_pose);
    moveit_msgs::RobotTrajectory place_trajectory;
    double place_fraction = move_group.computeCartesianPath(place_waypoints, eef_step, jump_threshold, place_trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing (Cartesian path) (%.2f%% acheived)", place_fraction * 100.0);
    if (place_fraction > 0.5)
    {
      move_group.execute(place_trajectory);
      gripper_group.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.0);
      gripper_group.move();
    }
    else
      break;

    ros::shutdown();
  }
  ros::shutdown();
  return 0;
}
