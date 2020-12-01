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
#include <discretized_movement/Discretized_Movement_Param_Server.hpp>


class Discretized_Movement_Action
{
  protected:

    actionlib::SimpleActionServer<discretized_movement::MoveAction> MoveActionServer_;
    int grid_steps_x, grid_steps_y;
    BoundingBox bounds;
    int current_x_coord, current_y_coord;
    std::vector<std::string> **world_states;
    geometry_msgs::Pose ee_start_pose;
    moveit::planning_interface::MoveGroupInterface *move_group;

  public:

    Discretized_Movement_Action(
        std::string name,
        ros::NodeHandle nh,
        moveit::planning_interface::MoveGroupInterface &move_group_
        ) :
      MoveActionServer_(nh, name,
          boost::bind(&Discretized_Movement_Action::execute, this, _1),
          false) {

        DiscretizedMovementParamServer paramServer(nh);
        bounds = paramServer.get_bounding_box();
        grid_steps_x = 13;
        grid_steps_y = 13;
        current_x_coord = 0;
        current_y_coord = 0;

        world_states = new std::vector<std::string>* [grid_steps_x];
        for(int n = 0; n < grid_steps_x; ++n)
            world_states[n] = new std::vector<std::string> [grid_steps_y];

        move_group = &move_group_;

        ee_start_pose = move_group->getCurrentPose().pose;

        MoveActionServer_.start();
      }


    ~Discretized_Movement_Action(void) {}


    void execute(const discretized_movement::MoveGoalConstPtr &goal) {
      ros::Rate r(1);

      int goal_coord_x = current_x_coord;
      int goal_coord_y = current_y_coord;
      // get goal direction and adjust goal state accordingly
      if(goal->move.direction == goal->move.UP) {
        goal_coord_x++;
      }
      else if(goal->move.direction == goal->move.DOWN) {
        goal_coord_x--;
      }
      else if(goal->move.direction == goal->move.LEFT) {
        goal_coord_y++;
      }
      else if(goal->move.direction == goal->move.RIGHT) {
        goal_coord_y--;
      }

      if(0 > goal_coord_y || goal_coord_y > grid_steps_y) {
        ROS_WARN("you tried going out-of-bounds!");
        return;
      }

      if(0 > goal_coord_x || goal_coord_x > grid_steps_y) {
        ROS_WARN("you tried going out-of-bounds!");
        return;
      }

      double goal_x = (goal_coord_x * step_size_x) + ee_start_pose.position.x;
      double goal_y = (goal_coord_y * step_size_y) + ee_start_pose.position.y;

      bool success = attempt_move(goal_x, goal_y);
      if(success) {
        current_x_coord = goal_coord_x;
        current_y_coord = goal_coord_y;
        MoveActionServer_.setSucceeded();
      } else {
        MoveActionServer_.setAborted();
      }
    }

    bool attempt_move(double goal_x, double goal_y) {
      std::vector<geometry_msgs::Pose> waypoints;
      geometry_msgs::Pose goal_pose = move_group->getCurrentPose().pose;
      goal_pose.position.x = goal_x;
      goal_pose.position.y = goal_y;

      ROS_INFO("Told to go to [%f, %f, %f][%f, %f, %f, %f]",
              goal_pose.position.x,
              goal_pose.position.y,
              goal_pose.position.z,
              goal_pose.orientation.x,
              goal_pose.orientation.y,
              goal_pose.orientation.z,
              goal_pose.orientation.w
              );

      move_group->setStartStateToCurrentState();
      move_group->setPoseTarget(goal_pose);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if(success) {
        move_group->move();
      }

      return success;
    }
};
