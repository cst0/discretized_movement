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
#include <std_msgs/String.h>
#include <sstream>

// ros stuff
#include <ros/ros.h>
#include <ros/console.h>

// actionlib server imports
#include <actionlib/server/simple_action_server.h>
#include <discretized_movement/MoveAction.h>
#include <discretized_movement/InteractAction.h>

// message imports
#include <discretized_movement/state.h>
#include <sensor_msgs/JointState.h>

// moveit stuff
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/GetPositionIK.h>

#include <discretized_movement/Bounding_Box.hpp>
#include <discretized_movement/Discretized_Interact_Action.hpp>
#include <discretized_movement/Discretized_Movement_Action.hpp>
#include <discretized_movement/Discretized_Movement_Param_Server.hpp>
#include <discretized_movement/World_Object.hpp>
#include <discretized_movement/World_State.hpp>
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

  for (int n = 0; n < (int) joint_group_positions.size(); ++n) {
    if (pose.count(names[n])) { // names[n] is in the map
      joint_group_positions[n] = pose[names[n]];
    }
  }

  move_group.setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan start_pose_plan;

  bool success = (move_group.plan(start_pose_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Moving to initial start pose planning resulted in %s", success ? "success" : "failed");
  if(success)
      success = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  return success;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "simplified_kinematics");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ROS_INFO("Starting up...");

  DiscretizedMovementParamServer param_server(nh);
  moveit::planning_interface::MoveGroupInterface move_group(param_server.get_group_name());

  Discretized_Movement_Action action_server("simplified_kinematics", nh, move_group);
  ROS_INFO("Movement action server is now running.");
  Discretized_Interact_Action interact_server("simplified_interaction", nh);
  ROS_INFO("Interaction action server is now running.");

  ROS_INFO("Going to try going to the start pose...");
  if(!goToPose(param_server.get_start_pose(), param_server.get_group_name(), move_group)) {
    ROS_ERROR("Unable to achieve start pose. Ending.");
    exit(2);
  }

  ROS_INFO("The entire node is now ready.");
  ros::waitForShutdown();

  ROS_INFO("Told to shut down. Goodbye!");

  return 0;
}
