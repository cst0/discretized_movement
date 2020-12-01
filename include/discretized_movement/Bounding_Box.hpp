#pragma once

class BoundingBox
{
  public:
    double step_size_x, step_size_y, max_x, max_y, min_x, min_y, start_x, start_y;

    BoundingBox(
        double step_size_x_,
        double step_size_y_,
        double max_x_,
        double max_y_,
        double min_x_,
        double min_y_,
        double start_x_,
        double start_y_)
    {
      step_size_x = step_size_x_;
      step_size_y = step_size_y_;
      max_x       = max_x_;
      max_y       = max_y_;
      min_x       = min_x_;
      min_y       = min_y_;
      start_x     = start_x_;
      start_y     = start_y_;
    }

    BoundingBox() {}
};
