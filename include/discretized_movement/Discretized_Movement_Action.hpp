#pragma once
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
#include <discretized_movement/Standard_Parameter_Names.h>

class Discretized_Movement_Action {
protected:
  actionlib::SimpleActionServer<discretized_movement::MoveAction>
      MoveActionServer_;

  discretized_movement::MoveFeedback feedback_;
  discretized_movement::MoveResult result_;

  BoundingBox bounds;
  int grid_steps_x, grid_steps_y;
  int current_x_coord, current_y_coord;
  int start_x_coord, start_y_coord;
  geometry_msgs::Pose inventory_return_pose;
  float ee_start_x, ee_start_y, ee_start_z;
  float inventory_x, inventory_y;

  std::mutex *world_state_mutex;
  discretized_movement::worldstate *world_state;
  moveit::planning_interface::MoveGroupInterface *move_group;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

public:
  Discretized_Movement_Action(
      std::string name, ros::NodeHandle nh,
      moveit::planning_interface::MoveGroupInterface &move_group_,
      std::mutex &m, discretized_movement::worldstate &world_state_)
      : MoveActionServer_(
            nh, name,
            boost::bind(&Discretized_Movement_Action::execute, this, _1),
            false) {

    DiscretizedMovementParamServer paramServer(nh);
    bounds = paramServer.get_bounding_box();
    grid_steps_x = 10;
    grid_steps_y = 10;
    start_x_coord = 5;
    start_y_coord = 5;
    current_x_coord = start_x_coord;
    current_y_coord = start_y_coord;

    world_state_mutex = &m;
    world_state = &world_state_;

    // basic init stuff
    move_group = &move_group_;
    //ee_start_pose = move_group->getCurrentPose().pose;
    ee_start_x = 0.6;
    ee_start_y = 0.0;
    ee_start_z = 1.074;

    inventory_x = 0.6;
    inventory_y = 0.35;

    paramServer.insert_obstacle(move_group);

    MoveActionServer_.start();
  }

  ~Discretized_Movement_Action(void) {}

  void execute(const discretized_movement::MoveGoalConstPtr &goal) {
    ros::Rate r(1);

    int goal_coord_x = current_x_coord;
    int goal_coord_y = current_y_coord;
    // get goal direction and adjust goal state accordingly
    if (goal->move.direction == goal->move.UP) {
      goal_coord_x++;
    } else if (goal->move.direction == goal->move.DOWN) {
      goal_coord_x--;
    } else if (goal->move.direction == goal->move.LEFT) {
      goal_coord_y++;
    } else if (goal->move.direction == goal->move.RIGHT) {
      goal_coord_y--;
    }

    if (0 > goal_coord_y || goal_coord_y > grid_steps_y) {
      ROS_WARN("you tried going out-of-bounds!");
      MoveActionServer_.setAborted();
      return;
    }

    if (0 > goal_coord_x || goal_coord_x > grid_steps_y) {
      ROS_WARN("you tried going out-of-bounds!");
      MoveActionServer_.setAborted();
      return;
    }

    //ROS_INFO("%f, %d, %f, %f", goal_y, goal_coord_y, bounds.step_size_y, ee_start_pose.position.y);

    world_state_mutex->lock();

    current_x_coord = goal_coord_x;
    current_y_coord = goal_coord_y;
    world_state->robot_state.x = current_x_coord;
    world_state->robot_state.y = current_y_coord;

    if(world_state->robot_state.grasping) {
        for (int n = 0; n < (int) world_state->observed_objects.size(); ++n) {
            if (world_state->observed_objects[n].name == world_state->robot_state.current_grasp) {
              world_state->observed_objects[n].x = current_x_coord;
              world_state->observed_objects[n].y = current_y_coord;
            }
        }
    }

    feedback_.worldstate = *world_state;
    result_.success = true;

    world_state_mutex->unlock();

    MoveActionServer_.publishFeedback(feedback_);
    MoveActionServer_.setSucceeded(result_);
  }

  bool move_from_buffer() {
    double goal_x = ee_start_x + ((current_x_coord-start_x_coord) * bounds.step_size_x);
        //(goal_coord_x * bounds.step_size_x) + ee_start_pose.position.x;
    double goal_y = ee_start_y + ((current_y_coord-start_y_coord) * bounds.step_size_y);
        //(goal_coord_y * bounds.step_size_y) + ee_start_pose.position.y;

    return attempt_move(goal_x, goal_y);
  }

  bool attempt_move(double goal_x, double goal_y) {
    geometry_msgs::Pose goal_pose = move_group->getCurrentPose().pose;
    goal_pose.position.x = goal_x;
    goal_pose.position.y = goal_y;
    goal_pose.position.z = ee_start_z;

    goal_pose.orientation.x = -0.003;
    goal_pose.orientation.y = 0.697;
    goal_pose.orientation.z = 0.001;
    goal_pose.orientation.w = 0.7161;

    ROS_INFO("Told to go to [%f, %f, %f][%f, %f, %f, %f]", goal_pose.position.x,
             goal_pose.position.y, goal_pose.position.z,
             goal_pose.orientation.x, goal_pose.orientation.y,
             goal_pose.orientation.z, goal_pose.orientation.w);

    move_group->setStartStateToCurrentState();
    move_group->setPoseTarget(goal_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group->plan(plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
      move_group->move();
    }

    return success;
  }

  bool move_to_inventory() {
    inventory_return_pose = move_group->getCurrentPose().pose;
    geometry_msgs::Pose inventory_pose = move_group->getCurrentPose().pose;

    // Go above the inventory
    inventory_pose.position.x = inventory_x;
    inventory_pose.position.y = inventory_y;
    move_group->setStartStateToCurrentState();
    move_group->setPoseTarget(inventory_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan_inventory;
    bool success = (move_group->plan(plan_inventory) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
      move_group->move();
      inventory_x -= 0.075; // this slot is full
    }
    return success;
  }

  bool return_from_inventory() {
    // Go back to the start
    move_group->setStartStateToCurrentState();
    move_group->setPoseTarget(inventory_return_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan_start;
    bool success = (move_group->plan(plan_start) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
      move_group->move();
    }

    return success;

  }
};
