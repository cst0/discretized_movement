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


class DiscretizedMovementParamServer
{
  protected:
    std::vector<double>       start_pose_joint_state;
    std::vector<std::string>  start_pose_joint_names;
    std::string               startup_group_name;
    std::string               runtime_group_name;
    bool                      have_group_names;
    BoundingBox               bounding_box;
    ros::NodeHandle           nh;

  public:
    DiscretizedMovementParamServer(ros::NodeHandle nh_)
    {
      this->nh = nh_;
      bounding_box = get_bounding_box();
      have_group_names = false;
    }

    ~DiscretizedMovementParamServer() {}

    std::map<std::string, double> get_start_pose() {
    //std::vector<double> get_start_pose() {
      if(
          nh.hasParam(ROSPARAM_NAME_STARTPOSE_JOINTSTATE) &&
          nh.hasParam(ROSPARAM_NAME_STARTPOSE_JOINTNAMES)
        ) {
        nh.getParam(ROSPARAM_NAME_STARTPOSE_JOINTSTATE, start_pose_joint_state);
        nh.getParam(ROSPARAM_NAME_STARTPOSE_JOINTNAMES, start_pose_joint_names);
      } else {
        ROS_ERROR("Joint state parameters must be set. Aborting.");
        exit(1);
      }

      if(start_pose_joint_names.size() != start_pose_joint_state.size()) {
        ROS_ERROR("Size mismatch between joint states and joint names, so states cannot be obtained. Aborting.");
        exit(3);
      }

      std::map<std::string, double> returnme;
      for(int n = 0; n < (int) start_pose_joint_names.size(); ++n) {
        returnme.insert(std::pair<std::string, double>(start_pose_joint_names.at(n), start_pose_joint_state.at(n)));
      }
      return returnme;
    }


   void server_get_group_names() {
      if(
          nh.hasParam(ROSPARAM_NAME_RUNTIME_GROUP_NAME) &&
          nh.hasParam(ROSPARAM_NAME_STARTUP_GROUP_NAME)
        ) {
        nh.getParam(ROSPARAM_NAME_RUNTIME_GROUP_NAME, runtime_group_name);
        nh.getParam(ROSPARAM_NAME_STARTUP_GROUP_NAME, startup_group_name);
      } else {
        ROS_ERROR("group name parameter must be set. Aborting.");
        exit(1);
      }
      have_group_names = true;
    }


   std::string get_runtime_group_name() {
     if (!have_group_names)
       server_get_group_names();
     return runtime_group_name;
   }


   std::string get_startup_group_name() {
     if (!have_group_names)
       server_get_group_names();
     return startup_group_name;
   }


    BoundingBox get_bounding_box() {
      double step_size, max_x, max_y, min_x, min_y, start_x, start_y;
      nh.param(ROSPARAM_NAME_STEP_SIZE,  step_size,  0.05);
      nh.param(ROSPARAM_NAME_MAX_X,      max_x,      0.65);
      nh.param(ROSPARAM_NAME_MIN_X,      min_x,      0.0);
      nh.param(ROSPARAM_NAME_START_X,    start_x,    0.0);
      nh.param(ROSPARAM_NAME_MAX_Y,      max_y,      0.65);
      nh.param(ROSPARAM_NAME_MIN_Y,      min_y,      0.0);
      nh.param(ROSPARAM_NAME_START_Y,    start_y,    0.0);

      return BoundingBox(step_size, max_x, max_y, min_x, min_y, start_x, start_y);

    }
};


