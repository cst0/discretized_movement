#ifndef __DISCRETIZED_SPACE_UPDATER_HPP
#define __DISCRETIZED_SPACE_UPDATER_HPP
#include "discretized_movement/worldstate.h"
#include "ros/ros.h"
#include <mutex>

class DiscretizedSpaceUpdater {
protected:
  std::mutex *world_state_mutex;
  discretized_movement::worldstate *world_state;

public:
  DiscretizedSpaceUpdater(std::mutex &m,
                          discretized_movement::worldstate &world_state_,
                          ros::NodeHandle &nh_) {
    world_state_mutex = &m;
    world_state = &world_state_;
    ros::Subscriber sub = nh_.subscribe("world_state_status", 1, &DiscretizedSpaceUpdater::spaceUpdaterCallback, this);
  }

  void spaceUpdaterCallback(const discretized_movement::worldstateConstPtr &msg) {
      world_state_mutex->lock();
      world_state->observed_objects = msg->observed_objects;
      world_state_mutex->unlock();
  }
};

#endif
