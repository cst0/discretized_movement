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
#include <discretized_movement/World_Object.hpp>
#include <discretized_movement/World_State.hpp>
#include <discretized_movement/Standard_Parameter_Names.h>


class DiscretizedMovementParamServer
{
  protected:
    std::vector<double>       start_pose_joint_state;
    std::vector<std::string>  start_pose_joint_names;
    std::string               group_name;
    BoundingBox               bounding_box;
    ros::NodeHandle           nh;

  public:
    DiscretizedMovementParamServer(ros::NodeHandle nh_)
    {
      this->nh = nh_;
      bounding_box = get_bounding_box();
    }


    ~DiscretizedMovementParamServer() {}


    std::map<std::string, double> get_start_pose() {
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

      if(start_pose_joint_state.size() == 0) {
          ROS_ERROR("length of start pose joint states is zero");
          exit(4);
      }

      if(start_pose_joint_names.size() == 0) {
          ROS_ERROR("length of start pose joint names is zero");
          exit(4);
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


    std::string get_group_name() {
      if (nh.hasParam(ROSPARAM_NAME_GROUP_NAME)) {
        nh.getParam(ROSPARAM_NAME_GROUP_NAME, group_name);
        return group_name;
      }
      ROS_ERROR("No group name specified. Returning empty string.");
      return "";
    }


    BoundingBox get_bounding_box() {
      double step_size_x, step_size_y, max_x, max_y, min_x, min_y, start_x, start_y;
      nh.param(ROSPARAM_NAME_STEP_SIZE_X , step_size_x , 0.05);
      nh.param(ROSPARAM_NAME_STEP_SIZE_Y , step_size_y , 0.05);
      nh.param(ROSPARAM_NAME_MAX_X       , max_x       , 0.65);
      nh.param(ROSPARAM_NAME_MIN_X       , min_x       , 0.0);
      nh.param(ROSPARAM_NAME_START_X     , start_x     , 0.0);
      nh.param(ROSPARAM_NAME_MAX_Y       , max_y       , 0.65);
      nh.param(ROSPARAM_NAME_MIN_Y       , min_y       , 0.0);
      nh.param(ROSPARAM_NAME_START_Y     , start_y     , 0.0);

      return BoundingBox(step_size_x, step_size_y, max_x, max_y, min_x, min_y, start_x, start_y);
    }
};
