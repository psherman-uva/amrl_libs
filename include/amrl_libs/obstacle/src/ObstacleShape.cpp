
#include <amrl_libs/obstacle/ObstacleShape.hpp>

namespace amrl {
  
  
ObstacleShape::ObstacleShape(const ShapeType &shape_type)
  : _type(shape_type)
{
}

ObstacleShape::ShapeType ObstacleShape::shape_type(void) const
{
  return _type;
}

void ObstacleShape::min_max_coordinates(double &min_x, double &max_x, double &min_y, double &max_y) const
{
  min_x = _min_X;
  max_x = _max_X;
  min_y = _min_Y;
  max_y = _max_Y;
}

}