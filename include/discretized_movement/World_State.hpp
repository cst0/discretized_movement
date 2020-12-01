#pragma once
#include <discretized_movement/Bounding_Box.hpp>

class WorldState
{
  public:
    BoundingBox bounding_box;
    std::vector<WorldObject> objects;

    WorldState(BoundingBox bounding_box_, std::vector<WorldObject> objects_) {
      bounding_box = bounding_box_;
      objects = objects_;
    }
};
