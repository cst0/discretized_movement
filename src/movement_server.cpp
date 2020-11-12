/*
 * TODO:
 * -refactor main into more functions
 * -refactor this into mulitple files
 */

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

#include <discretized_movement/standard_parameter_names.h>

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

      result_.success = attempt_move();
      if(result_.success) {
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
      return false;
    }
};


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


bool goToPose(std::vector<double> pose,
              std::string planning_group,
              moveit::planning_interface::MoveGroupInterface& move_group)
{
  sensor_msgs::JointState msg;

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> current_joint_positions;
  current_state->copyJointGroupPositions(planning_group, current_joint_positions);

  if(current_joint_positions.size() == pose.size()) {
    move_group.setJointValueTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    return move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  }
  else {
    ROS_ERROR("Cannot go to start pose: different number of actual joint states and goal states.");
    return false;
  }

  return false;
}


class BoundingBox
{
  public:
    double step_size, max_x, max_y, min_x, min_y, start_x, start_y;

    BoundingBox(
        double step_size_,
        double max_x_,
        double max_y_,
        double min_x_,
        double min_y_,
        double start_x_,
        double start_y_)
    {
      step_size = step_size_;
      max_x     = max_x_;
      max_y     = max_y_;
      min_x     = min_x_;
      min_y     = min_y_;
      start_x   = start_x_;
      start_y   = start_y_;
    }

    BoundingBox() {}
};


class DiscretizedMovementParamServer
{
  protected:
    std::vector<double>       start_pose_joint_states;
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

    std::vector<double> get_start_pose() {
      if(
          nh.hasParam(ROSPARAM_NAME_STARTPOSE_JOINTSTATE)
        ) {
        nh.param(ROSPARAM_NAME_STARTPOSE_JOINTSTATE, start_pose_joint_states);
      } else {
        ROS_ERROR("Joint state parameter must be set. Aborting.");
        exit(1);
      }
      return start_pose_joint_states;
    }

    std::string get_group_name() {
      if(
          nh.hasParam(ROSPARAM_NAME_GROUP_NAME)
        ) {
        nh.param(ROSPARAM_NAME_GROUP_NAME, group_name);
      } else {
        ROS_ERROR("group name parameter must be set. Aborting.");
        exit(1);
      }
      return group_name;
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


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "simplified_kinematics");
  ros::NodeHandle nh;
  ROS_INFO("Starting up...");

  Discretized_Movement_Action action_server("simplified_kinematics", nh);
  ROS_INFO("Movement action server is now running.");
  Discretized_Interact_Action interact_server("simplified_interaction", nh);
  ROS_INFO("Interaction action server is now running.");

  ROS_INFO("Going to try going to the start pose...");
  DiscretizedMovementParamServer param_server(nh);
  moveit::planning_interface::MoveGroupInterface move_group(param_server.get_group_name());

  if(!goToPose(param_server.get_start_pose(), param_server.get_group_name(), move_group)) {
    ROS_ERROR("Unable to achieve start pose. Ending.");
    exit(2);
  }

  ROS_INFO("The entire node is now ready.");
  ros::spin();

  ROS_INFO("Told to shut down. Goodbye!");

  return 0;
}
