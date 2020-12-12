#pragma once
// standard includes
#include <mutex>
#include <sstream>
#include <std_msgs/String.h>

// ros stuff
#include <discretized_movement/worldstate.h>
#include <ros/console.h>
#include <ros/ros.h>

// actionlib server imports
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <discretized_movement/InteractAction.h>

// message imports
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
  bool do_not_connect;

public:
  Discretized_Interact_Action(std::string name, ros::NodeHandle nh,
                              std::mutex &m,
                              discretized_movement::worldstate &world_state_,
                              bool do_not_connect_)
      : InteractActionServer_(
            nh, name,
            boost::bind(&Discretized_Interact_Action::execute, this, _1),
            false),
        gripper_client("gripper_controller/gripper_action"), action_name_(name),
        paramServer(nh) {
    do_not_connect = do_not_connect_;
    world_state = &world_state_;
    world_state_mutex = &m;

    InteractActionServer_.start();
  }

  ~Discretized_Interact_Action(void) {}

  void attach_move_group(
      moveit::planning_interface::MoveGroupInterface &move_group_) {
    move_group = &move_group_;
  }

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
    if (!do_not_connect)
      paramServer.remove_obstacles();
    world_state_mutex->lock();
    feedback_.worldstate = *world_state;
    if (world_state->robot_state.grasping) {
      ROS_INFO("You're already grasping an object! Won't try grasping again.");
      world_state_mutex->unlock();
      if (!do_not_connect)
        paramServer.reinsert_obstacles(move_group);
      result_.success = false;
      return false;
    }

    for (int n = 0; n < (int)world_state->observed_objects.size(); ++n) {
      int max_layer_index = 0;
      if (world_state->observed_objects[n].x == world_state->robot_state.x &&
          world_state->observed_objects[n].y == world_state->robot_state.y) {
        for (int m = 0; m < (int)world_state->observed_objects.size(); ++m) {
          if (world_state->observed_objects[m].x ==
                  world_state->robot_state.x &&
              world_state->observed_objects[m].y ==
                  world_state->robot_state.y) {
            if (world_state->observed_objects[m].layer >=
                world_state->observed_objects[max_layer_index].layer)
              max_layer_index = m;
          }
        }

        ROS_INFO("Interacting with object: %d (%f, %f) named %s", n,
                 world_state->observed_objects[n].x,
                 world_state->observed_objects[n].y,
                 world_state->observed_objects[n].name.c_str());
        open_gripper();
        go_down(world_state->observed_objects[max_layer_index].layer);
        close_gripper();
        go_up();
        world_state->robot_state.grasping = true;
        world_state->robot_state.current_grasp =
            world_state->observed_objects[n].name;
        world_state->observed_objects[n].grasped = true;
        world_state_mutex->unlock();
        if (!do_not_connect)
          paramServer.reinsert_obstacles(move_group);
        feedback_.worldstate = *world_state;
        result_.success = true;
        ROS_INFO("Grabbed!");
        return true;
      }
    }
    world_state_mutex->unlock();

    if (!do_not_connect)
      paramServer.reinsert_obstacles(move_group);
    result_.success = false;
    ROS_INFO("You tried grasping at a point where there was nothing to grasp!");
    return false;
  }

  bool attempt_release() {
    if (!do_not_connect)
      paramServer.remove_obstacles();
    world_state_mutex->lock();
    feedback_.worldstate = *world_state;
    if (!world_state->robot_state.grasping) {
      ROS_INFO("You're not grasping any object! Nothing to try to release.");
      world_state_mutex->unlock();
      result_.success = false;
      if (!do_not_connect)
        paramServer.reinsert_obstacles(move_group);
      return false;
    }

    for (int n = 0; n < (int)world_state->observed_objects.size(); ++n) {
      if (world_state->observed_objects[n].name ==
          world_state->robot_state.current_grasp) {
        double layer = -1; // starting at -1 since there will be one block at
                           // the current position (in the gripper)
        for (int m = 0; m < (int)world_state->observed_objects.size(); ++m) {
          if (world_state->observed_objects[m].x ==
                  world_state->robot_state.x &&
              world_state->observed_objects[m].y == world_state->robot_state.y)
            layer++;
        }

        go_down(layer);
        open_gripper();
        go_up();

        world_state->robot_state.current_grasp = "";
        world_state->robot_state.grasping = false;
        world_state->observed_objects[n].grasped = false;
        world_state->observed_objects[n].layer = layer;
        feedback_.worldstate = *world_state;
        result_.success = true;
        world_state_mutex->unlock();
        if (!do_not_connect)
          paramServer.reinsert_obstacles(move_group);
        ROS_INFO("Released!");
        return true;
      }
    }

    world_state_mutex->unlock();
    if (!do_not_connect)
      paramServer.reinsert_obstacles(move_group);
    result_.success = false;
    return false;
  }

  bool open_gripper() {
    ROS_INFO("told to open");
    return move_to_position(.5);
  }

  bool close_gripper() {
    ROS_INFO("told to close");
    return move_to_position(0);
  }

  void go_down(int layer) {
    if (do_not_connect)
      return;
    layer += 1;
    double layer_height = paramServer.get_layer_height();
    double table_height = paramServer.get_table_height();

    geometry_msgs::PoseStamped pose = move_group->getCurrentPose();
    pose.pose.position.z =
        table_height + (layer_height * layer) + (0.90 - table_height);
    // ROS_INFO("layer %d (%f off of table at %f), %f real", layer,
    // layer_height, table_height, pose.pose.position.z);
    move_group->setPoseTarget(pose);
    move_group->move();
  }

  void go_up() {
    if (do_not_connect)
      return;
    double table_height = paramServer.get_table_height();
    double layer_height = paramServer.get_layer_height();
    double layer_count = paramServer.get_layer_count();
    double z = table_height + (layer_height * layer_count);

    geometry_msgs::PoseStamped pose = move_group->getCurrentPose();
    pose.pose.position.z = 1.058; // z;
    // ROS_INFO("layer %f (%f off of table), %f real", layer_count,
    // layer_height, pose.pose.position.z);
    move_group->setPoseTarget(pose);
    move_group->move();
  }

  bool move_to_position(double position) {
    if (do_not_connect)
      return true;
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
