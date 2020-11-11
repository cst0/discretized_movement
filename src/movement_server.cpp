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

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "simplified_kinematics");
  ros::NodeHandle nh;
  ROS_INFO("Starting up...");

  ros::Rate loop_rate(10);

  Discretized_Movement_Action action_server("simplified_kinematics", nh);
  ROS_INFO("Movement action server is now running.");
  Discretized_Interact_Action interact_server("simplified_interaction", nh);
  ROS_INFO("Interaction action server is now running.");
  ros::spin();

  ROS_INFO("Told to shut down. Goodbye!");

  return 0;
}
