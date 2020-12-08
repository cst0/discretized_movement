#pragma once
// standard includes
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
#include <discretized_movement/state.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

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
#include <discretized_movement/Standard_Parameter_Names.h>
#include <utility>

class DiscretizedMovementParamServer {
protected:
  std::vector<double> start_pose_joint_state;
  std::vector<std::string> start_pose_joint_names;
  std::string group_name;
  BoundingBox bounding_box;
  ros::NodeHandle nh;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

public:
  DiscretizedMovementParamServer(ros::NodeHandle nh_) {
    this->nh = nh_;
    bounding_box = get_bounding_box();
  }

  ~DiscretizedMovementParamServer() {}

  std::map<std::string, double> get_start_pose() {
    if (nh.hasParam(ROSPARAM_NAME_STARTPOSE_JOINTSTATE) &&
        nh.hasParam(ROSPARAM_NAME_STARTPOSE_JOINTNAMES)) {
      nh.getParam(ROSPARAM_NAME_STARTPOSE_JOINTSTATE, start_pose_joint_state);
      nh.getParam(ROSPARAM_NAME_STARTPOSE_JOINTNAMES, start_pose_joint_names);
    } else {
      ROS_ERROR("Joint state parameters must be set. Aborting.");
      exit(1);
    }

    if (start_pose_joint_state.size() == 0) {
      ROS_ERROR("length of start pose joint states is zero");
      exit(4);
    }

    if (start_pose_joint_names.size() == 0) {
      ROS_ERROR("length of start pose joint names is zero");
      exit(4);
    }

    if (start_pose_joint_names.size() != start_pose_joint_state.size()) {
      ROS_ERROR("Size mismatch between joint states and joint names, so states "
                "cannot be obtained. Aborting.");
      exit(3);
    }

    std::map<std::string, double> returnme;
    for (int n = 0; n < (int)start_pose_joint_names.size(); ++n) {
      returnme.insert(std::pair<std::string, double>(
          start_pose_joint_names.at(n), start_pose_joint_state.at(n)));
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

  double get_layer_height() {
    double returnval;
    nh.param(ROSPARAM_NAME_LAYER_HEIGHT, returnval, 0.04);
    return returnval;
  }

  double get_layer_count() {
    double returnval;
    nh.param(ROSPARAM_NAME_LAYER_COUNT, returnval, 5.0);
    return returnval;
  }

  BoundingBox get_bounding_box() {
    double step_size_x, step_size_y, max_x, max_y, min_x, min_y, start_x,
        start_y;
    nh.param(ROSPARAM_NAME_STEP_SIZE_X, step_size_x, 0.05);
    nh.param(ROSPARAM_NAME_STEP_SIZE_Y, step_size_y, 0.05);
    nh.param(ROSPARAM_NAME_MAX_X, max_x, 0.65);
    nh.param(ROSPARAM_NAME_MIN_X, min_x, 0.0);
    nh.param(ROSPARAM_NAME_START_X, start_x, 0.0);
    nh.param(ROSPARAM_NAME_MAX_Y, max_y, 0.65);
    nh.param(ROSPARAM_NAME_MIN_Y, min_y, 0.0);
    nh.param(ROSPARAM_NAME_START_Y, start_y, 0.0);

    return BoundingBox(step_size_x, step_size_y, max_x, max_y, min_x, min_y,
                       start_x, start_y);
  }

  double get_table_height() {
    return get_layer_height(ROSPARAM_NAME_TABLE_HEIGHT);
  }

  double get_obstacle_height() {
    return get_layer_height(ROSPARAM_NAME_OBSTACLE_HEIGHT);
  }

  double get_layer_height(std::string layer_name) {
    double returnme = 0;
    if (nh.hasParam(layer_name))
      nh.getParam(layer_name, returnme);

    if (returnme == 0)
      ROS_ERROR("%s set to 0, has it been explicitly set properly?",
                layer_name.c_str());

    return returnme;
  }

  std::pair<geometry_msgs::Pose, shape_msgs::SolidPrimitive> get_obstacle(std::string obstacle) {
    BoundingBox bb = get_bounding_box();
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = bb.max_x - bb.min_x;
    primitive.dimensions[1] = bb.max_y - bb.min_y;
    primitive.dimensions[2] = get_layer_height(obstacle);

    geometry_msgs::Pose position;
    position.orientation.w = 1;
    position.position.x = bb.start_x + (primitive.dimensions[0]/2);
    position.position.y = bb.start_y + (primitive.dimensions[1]/2);

    position.position.z = primitive.dimensions[2]/2; // table height is handled by box size

    return std::pair<geometry_msgs::Pose, shape_msgs::SolidPrimitive>(position, primitive);
  }

  std::pair<geometry_msgs::Pose, shape_msgs::SolidPrimitive> get_table_obstacle() {
    return get_obstacle(ROSPARAM_NAME_TABLE_HEIGHT);
  }

  std::pair<geometry_msgs::Pose, shape_msgs::SolidPrimitive> get_obstacle_obstacle() {
    return get_obstacle(ROSPARAM_NAME_OBSTACLE_HEIGHT);
  }

  moveit_msgs::CollisionObject get_table_obstacle(moveit::planning_interface::MoveGroupInterface &move_group) {
      moveit_msgs::CollisionObject co;
      co.header.frame_id = move_group.getPlanningFrame();
      co.id = "table";

      std::pair<geometry_msgs::Pose, shape_msgs::SolidPrimitive> table_pair = get_table_obstacle();

      co.primitive_poses.push_back(table_pair.first);
      co.primitives.push_back(table_pair.second);
      co.operation = co.ADD;

      return co;
  }

  moveit_msgs::CollisionObject get_obstacle_obstacle(moveit::planning_interface::MoveGroupInterface &move_group) {
      moveit_msgs::CollisionObject co;
      co.header.frame_id = move_group.getPlanningFrame();
      co.id = "obstacle_layer";

      std::pair<geometry_msgs::Pose, shape_msgs::SolidPrimitive> table_pair = get_obstacle_obstacle();

      co.primitive_poses.push_back(table_pair.first);
      co.primitives.push_back(table_pair.second);
      co.operation = co.ADD;

      return co;
  }

  void insert_obstacle(moveit::planning_interface::MoveGroupInterface *move_group) {
        // set up table, obstacle layer obstacle
        std::vector<moveit_msgs::CollisionObject> vec;
        vec.push_back(get_table_obstacle(*move_group));
        vec.push_back(get_obstacle_obstacle(*move_group));
        planning_scene_interface.addCollisionObjects(vec);

  }

  void remove_table_obstacle() {
      std::vector<std::string> vec;
      vec.push_back("obstacle_layer");
      planning_scene_interface.removeCollisionObjects(vec);
  }

  void reinsert_table_obstacle(moveit::planning_interface::MoveGroupInterface *move_group) {
      std::vector<moveit_msgs::CollisionObject> vec;
      vec.push_back(get_table_obstacle(*move_group));
      planning_scene_interface.addCollisionObjects(vec);
  }

};
