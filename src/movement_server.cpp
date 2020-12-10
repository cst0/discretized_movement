/**
 * This code is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this codebase.  If not, see <https://www.gnu.org/licenses/>.
 *
 * Copyright cst (chris thierauf, <chris@cthierauf.com>) 2020, all rights
 * reserved.
 */

// standard includes
#include <mutex>
#include <sstream>
#include <std_msgs/String.h>

// ros stuff
#include <ros/console.h>
#include <ros/ros.h>

// actionlib server imports
#include <actionlib/server/simple_action_server.h>
#include <discretized_movement/InteractAction.h>
#include <discretized_movement/MoveAction.h>
#include <discretized_movement/worldobject.h>

// message imports
#include <discretized_movement/Discretized_Space_Updater.hpp>
#include <discretized_movement/worldstate.h>
#include <sensor_msgs/JointState.h>

// moveit stuff
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/GetPositionIK.h>

#include <discretized_movement/Bounding_Box.hpp>
#include <discretized_movement/Discretized_Interact_Action.hpp>
#include <discretized_movement/Discretized_Movement_Action.hpp>
#include <discretized_movement/Discretized_Movement_Param_Server.hpp>
#include <discretized_movement/Standard_Parameter_Names.h>

bool goToPose(std::map<std::string, double> pose, std::string planning_group,
              moveit::planning_interface::MoveGroupInterface &move_group) {
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  const robot_state::JointModelGroup *joint_model_group =
      current_state->getJointModelGroup(planning_group);
  current_state->copyJointGroupPositions(joint_model_group,
                                         joint_group_positions);
  std::vector<std::string> names = joint_model_group->getJointModelNames();

  for (int n = 0; n < (int)joint_group_positions.size(); ++n) {
    if (pose.count(names[n])) { // names[n] is in the map
      joint_group_positions[n] = pose[names[n]];
    }
  }

  move_group.setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan start_pose_plan;

  bool success = (move_group.plan(start_pose_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Moving to initial start pose planning resulted in %s",
           success ? "success" : "failed");
  if (success)
    success = (move_group.move() ==
               moveit::planning_interface::MoveItErrorCode::SUCCESS);
  return success;
}

discretized_movement::worldstate world_state;
std::mutex world_state_mutex;
void spaceUpdaterCallback(const discretized_movement::worldstateConstPtr &msg) {
  world_state_mutex.lock();
  world_state.observed_objects = msg->observed_objects;
  world_state_mutex.unlock();
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "simplified_kinematics");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(0);
  std::vector<std::basic_string<char>> newargv;
  ros::removeROSArgs(argc, argv, newargv);

  spinner.start();
  ROS_INFO("Starting up...");

  ros::Rate rate(10);
  ros::Subscriber sub =
      nh.subscribe("/world_state_status", 1, spaceUpdaterCallback);

  DiscretizedMovementParamServer param_server(nh);
  bool do_not_connect = param_server.get_do_not_connect();

  Discretized_Movement_Action action_server("simplified_kinematics", nh,
                                            world_state_mutex, world_state,
                                            do_not_connect);
  ROS_INFO("Movement action server is now running.");
  Discretized_Interact_Action interact_server("simplified_interaction", nh,
                                              world_state_mutex, world_state,
                                              do_not_connect);
  ROS_INFO("Interaction action server is now running.");

  if (!do_not_connect) {
    ROS_INFO("Attempting to attach to move group.");
    moveit::planning_interface::MoveGroupInterface move_group(
        param_server.get_group_name());
    action_server.attach_move_group(move_group);
    interact_server.attach_move_group(move_group);

    ROS_INFO("Going to try going to the start pose...");
    geometry_msgs::PoseStamped p = move_group.getCurrentPose();
    bool pose_success = goToPose(param_server.get_start_pose(),
                                 param_server.get_group_name(), move_group);
    if (!pose_success) {
      ROS_ERROR("Unable to achieve start pose. Ending.");
      exit(2);
    }
  } else {
    ROS_INFO(
        "Did not attempt to attach to move group, because do_not_connect=true");
  }

  ros::Publisher pub =
      nh.advertise<discretized_movement::worldstate>("world_state", 5);
  ROS_INFO("The entire node is now ready.");
  while (ros::ok()) {
    pub.publish(world_state);
    rate.sleep();
  }

  ROS_INFO("Told to shut down. Goodbye!");

  return 0;
}
