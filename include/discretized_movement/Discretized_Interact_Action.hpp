#pragma once
// standard includes
#include <std_msgs/String.h>
#include <sstream>

// ros stuff
#include <ros/ros.h>
#include <ros/console.h>

// actionlib server imports
#include <actionlib/server/simple_action_server.h>
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
#include <discretized_movement/Discretized_Movement_Param_Server.hpp>
#include <discretized_movement/World_Object.hpp>
#include <discretized_movement/World_State.hpp>
#include <discretized_movement/Standard_Parameter_Names.h>


class Discretized_Interact_Action
{

  protected:

    actionlib::SimpleActionServer<discretized_movement::InteractAction> InteractActionServer_;
    std::string action_name_;
    discretized_movement::InteractFeedback feedback_;
    discretized_movement::InteractResult result_;

  public:
    Discretized_Interact_Action(std::string name, ros::NodeHandle nh) :
      InteractActionServer_(nh, name,
          boost::bind(&Discretized_Interact_Action::execute, this, _1),
          false),
      action_name_(name) {
        InteractActionServer_.start();
      }

    ~Discretized_Interact_Action(void) {}

    void execute(const discretized_movement::InteractGoalConstPtr &goal) {
      bool success = false;
      if(goal->action.interact == goal->action.GRAB)
        success = attempt_grab();
      else if(goal->action.interact == goal->action.RELEASE)
        success = attempt_release();
      else
        ROS_ERROR("Invalid interaction specification");

      if(success)
        InteractActionServer_.setSucceeded();
      else
        InteractActionServer_.setAborted();

    }

    bool attempt_grab() {
      //TODO
      ROS_ERROR("grab action not yet implemented");
      return false;
    }

    bool attempt_release() {
      //TODO
      ROS_ERROR("release action not yet implemented");
      return false;
    }
};


