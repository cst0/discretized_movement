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
#include <mutex>

// ros stuff
#include <ros/ros.h>
#include <ros/console.h>

// actionlib server imports
#include <actionlib/server/simple_action_server.h>
#include <discretized_movement/MoveAction.h>
#include <discretized_movement/InteractAction.h>
#include <discretized_movement/worldobject.h>

// message imports
#include <discretized_movement/worldstate.h>
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

bool goToPose(double x, double y, double z, moveit::planning_interface::MoveGroupInterface &move_group) {
    move_group.setRPYTarget(0, 1.34, 0);
    return true;
}

void populate_world_state(discretized_movement::worldstate &world_state, std::mutex &m)
{
    m.lock();
    world_state.robot_state.grasping = false;
    world_state.robot_state.x = 0;
    world_state.robot_state.y = 0;
    discretized_movement::worldobject wo;
    wo.x = 0;
    wo.y = 0;
    discretized_movement::worldobject wo1;
    wo1.x = 2;
    wo1.y = 2;
    world_state.observed_objects.push_back(wo);
    world_state.observed_objects.push_back(wo1);
    m.unlock();
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

  std::mutex world_state_mutex;
  discretized_movement::worldstate world_state;
  populate_world_state(world_state, world_state_mutex);

  Discretized_Movement_Action action_server("simplified_kinematics", nh, move_group, world_state_mutex, world_state);
  ROS_INFO("Movement action server is now running.");
  Discretized_Interact_Action interact_server("simplified_interaction", nh, move_group, world_state_mutex, world_state);
  ROS_INFO("Interaction action server is now running.");

  ROS_INFO("Going to try going to the start pose...");
  geometry_msgs::PoseStamped p = move_group.getCurrentPose();
  if(
          !goToPose(param_server.get_start_pose(), param_server.get_group_name(), move_group) //||
          //!goToPose(
          //    param_server.get_bounding_box().start_x,
          //    param_server.get_bounding_box().start_y,
          //    (param_server.get_table_height() + (param_server.get_layer_height() * param_server.get_layer_count())),
          //    move_group
          //    )
          ) {
    ROS_ERROR("Unable to achieve start pose. Ending.");
    exit(2);
  }

  ros::Publisher pub = nh.advertise<discretized_movement::worldstate>("world_state", 5);
  ros::Rate rate(10);
  ROS_INFO("The entire node is now ready.");
  while(ros::ok()) {
      pub.publish(world_state);
      rate.sleep();
  }

  ROS_INFO("Told to shut down. Goodbye!");

  return 0;
}
