
#include <amrl_libs/mapping/GridInfo.hpp>

namespace amrl {

GridInfo::GridInfo(const Point<double> &origin,
    const double width,
    const double height,
    const double resolution)
  : origin(origin), width(width), height(height), resolution(resolution)
{
}

}