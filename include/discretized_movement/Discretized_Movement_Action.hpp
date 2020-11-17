#pragma once
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


class Discretized_Movement_Action
{
  protected:

    actionlib::SimpleActionServer<discretized_movement::MoveAction> MoveActionServer_;
    std::string action_name_;
    discretized_movement::MoveFeedback feedback_;
    discretized_movement::MoveResult result_;
    float step_size, goal_x, goal_y, current_x, current_y;

  public:

    Discretized_Movement_Action(std::string name, ros::NodeHandle nh) :
      MoveActionServer_(nh, name,
          boost::bind(&Discretized_Movement_Action::execute, this, _1),
          false),
      action_name_(name) {
    step_size = 0.05;
    goal_x = 0;
    goal_y = 0;
    current_x = 0;
    current_y = 0;
    MoveActionServer_.start();
  }


    ~Discretized_Movement_Action(void) {}


    void execute(const discretized_movement::MoveGoalConstPtr &goal) {
      ros::Rate r(1);

      // get goal direction and adjust goal state accordingly
      if(goal->move.direction == goal->move.RIGHT) {
        goal_x = feedback_.state.x + step_size;
      }
      else if(goal->move.direction == goal->move.RIGHT) {
        goal_x = feedback_.state.x - step_size;
      }
      else if(goal->move.direction == goal->move.RIGHT) {
        goal_y = feedback_.state.y + step_size;
      }
      else if(goal->move.direction == goal->move.RIGHT) {
        goal_y = feedback_.state.y - step_size;
      }

      bool success = attempt_move();
      result_.success = success;
      if(success) {
        feedback_.state.x = goal_x;
        feedback_.state.y = goal_y;
        MoveActionServer_.setSucceeded();
      } else {
        MoveActionServer_.setAborted();
      }
    }

    bool attempt_move() {
      //TODO
      ROS_ERROR("move function not implmeneted");
      return true;
    }
};


