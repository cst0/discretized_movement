#pragma once

class WorldObject
{
  public:
    std::string obj;
    double x;
    double y;
    double layer;
    bool grasping;

    WorldObject(std::string obj_, double x_, double y_, double layer_, bool grasping_) {
      obj      = obj_;
      x        = x_;
      y        = y_;
      layer    = layer_;
      grasping = grasping_;
    }
};
