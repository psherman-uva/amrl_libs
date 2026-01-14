/*
  @file:      GridInfo.hpp
  @author:    psherman
  @date       June 2024
  
  @brief Convenience struct for info for Grid initialization
*/

#pragma once

#include <amrl_common/point/Point.hpp>

namespace amrl {

struct GridInfo {

  GridInfo(void)  = default;
  ~GridInfo(void) = default;

  GridInfo(const Point<double> &origin,
    const double width,
    const double height,
    const double resolution);

  Point<double> origin;
  double width;
  double height;
  double resolution;
};

}

