#pragma once
// standard includes
#include <mutex>
#include <sstream>
#include <std_msgs/String.h>

// ros stuff
#include "discretized_movement/worldstate.h"
#include <ros/console.h>
#include <ros/ros.h>

// actionlib server imports
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <discretized_movement/InteractAction.h>

// message imports
#include <discretized_movement/state.h>
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
#include <discretized_movement/Discretized_Movement_Param_Server.hpp>

class Discretized_Interact_Action {

protected:
  actionlib::SimpleActionServer<discretized_movement::InteractAction>
      InteractActionServer_;
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction>
      gripper_client;
  std::string action_name_;
  discretized_movement::InteractFeedback feedback_;
  moveit::planning_interface::MoveGroupInterface *move_group;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  discretized_movement::InteractResult result_;
  discretized_movement::worldstate *world_state;
  DiscretizedMovementParamServer paramServer;

  std::mutex *world_state_mutex;

public:
  Discretized_Interact_Action(
      std::string name, ros::NodeHandle nh,
      moveit::planning_interface::MoveGroupInterface &move_group_,
      std::mutex &m, discretized_movement::worldstate &world_state_)
      : InteractActionServer_(
            nh, name,
            boost::bind(&Discretized_Interact_Action::execute, this, _1),
            false),
        gripper_client("gripper_controller/gripper_action"),
        action_name_(name),
        paramServer(nh)
    {
    move_group = &move_group_;
    world_state = &world_state_;
    world_state_mutex = &m;

    InteractActionServer_.start();
  }

  ~Discretized_Interact_Action(void) {}

  void execute(const discretized_movement::InteractGoalConstPtr &goal) {
    bool success = false;
    if (goal->action.interact == goal->action.GRAB) {
      success = attempt_grab();
    } else if (goal->action.interact == goal->action.RELEASE) {
      success = attempt_release();
    } else
      ROS_ERROR("Invalid interaction specification");

    result_.success = success;
    InteractActionServer_.setSucceeded(result_);
    InteractActionServer_.publishFeedback(feedback_);
  }

  bool attempt_grab() {
    world_state_mutex->lock();
    feedback_.worldstate = *world_state;
    if (world_state->robot_state.grasping) {
      ROS_INFO("You're already grasping an object! Won't try grasping again.");
      world_state_mutex->unlock();
      result_.success = false;
      return false;
    }

    for (int n = 0; n < (int)world_state->observed_objects.size(); ++n) {
      if (world_state->observed_objects[n].x == world_state->robot_state.x &&
          world_state->observed_objects[n].y == world_state->robot_state.y) {
        open_gripper();
        go_down(world_state->observed_objects[n].layer);
        close_gripper();
        go_up();
        world_state->robot_state.grasping = true;
        world_state->robot_state.current_grasp =
            world_state->observed_objects[n].name;
        world_state->observed_objects[n].grasped = true;
        world_state_mutex->unlock();
        feedback_.worldstate = *world_state;
        result_.success = true;
        ROS_INFO("Grabbed!");
        return true;
      }
    }
    world_state_mutex->unlock();

    result_.success = false;
    ROS_INFO("You tried grasping at a point where there was nothing to grasp!");
    return false;
  }

  bool attempt_release() {
    world_state_mutex->lock();
    feedback_.worldstate = *world_state;
    if (!world_state->robot_state.grasping) {
      ROS_INFO("You're not grasping any object! Nothing to try to release.");
      world_state_mutex->unlock();
      result_.success = false;
      return false;
    }

    for (int n = 0; n < (int)world_state->observed_objects.size(); ++n) {
      if (world_state->observed_objects[n].name ==
          world_state->robot_state.current_grasp) {
        go_down(world_state->observed_objects[n].layer);
        open_gripper();
        go_up();

        world_state->robot_state.current_grasp = "";
        world_state->robot_state.grasping = false;
        world_state->observed_objects[n].grasped = false;
        feedback_.worldstate = *world_state;
        result_.success = true;
        world_state_mutex->unlock();
        ROS_INFO("Released!");
        return true;
      }
    }

    world_state_mutex->unlock();
    result_.success = false;
    return false;
  }

  bool open_gripper() {
    ROS_INFO("told to open");
    return move_to_position(1);
  }

  bool close_gripper() {
    ROS_INFO("told to close");
    return move_to_position(0);
  }

  void go_down(int layer) {
      double z = paramServer.get_table_height();
      double layer_height = paramServer.get_layer_height();

      double descend = layer_height * layer;
      geometry_msgs::PoseStamped pose = move_group->getCurrentPose();
      pose.pose.position.z -= descend;
      move_group->setPoseTarget(pose);
      move_group->move();
  }

  void go_up() {
      double z = paramServer.get_table_height();
      double layer_height = paramServer.get_layer_height();

      double height = layer_height * paramServer.get_layer_count();
      geometry_msgs::PoseStamped pose = move_group->getCurrentPose();
      pose.pose.position.z = height + z;
      move_group->setPoseTarget(pose);
      move_group->move();
  }

  bool move_to_position(double position) {
    if (!gripper_client.isServerConnected()) {
      ROS_INFO("[FetchGripper] waitForServer...");
      gripper_client.waitForServer();
      ROS_INFO("... done");
    }

    control_msgs::GripperCommandGoal goal;
    goal.command.position = position;
    gripper_client.sendGoal(goal);

    ROS_INFO("[FetchGripper] waitForResult...");
    bool result = gripper_client.waitForResult();
    if (result) {
      actionlib::SimpleClientGoalState state = gripper_client.getState();
      ROS_INFO_STREAM("[FetchGripper] " << state.toString() << ": "
                                        << state.getText());
    } else {
      ROS_INFO("[FetchGripper] Action did not finish before the time out.");
    }
      return result;
  }
};
