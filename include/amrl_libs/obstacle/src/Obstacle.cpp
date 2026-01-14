
#include <amrl_libs/obstacle/Obstacle.hpp>
#include <amrl_common/util/util.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <limits>
#include <cmath>

namespace amrl {

Obstacle::Obstacle(const ObstacleType type, const std::vector<Point<double>> &coordinates) : 
  _pimpl(std::make_shared<PolygonObstacle>(coordinates)),
  _type(type)
{
}

Obstacle::Obstacle(const std::vector<Point<double>> &coordinates) : 
  Obstacle(ObstacleType::kSolid, coordinates)
{
}

Obstacle::Obstacle(const ObstacleType type, const Point<double> center, const double radius) :
  _pimpl(std::make_shared<CircleObstacle>(center, radius)),
  _type(type)
{
}

Obstacle::Obstacle(const Point<double> center, const double radius) :
  Obstacle(ObstacleType::kSolid, center, radius)
{
}

std::vector<Point<double>> Obstacle::coordinates(void) const
{
  std::vector<double> data = _pimpl->data();

  std::vector<Point<double>> coords(data.size()/2);
  for(size_t i = 0; i < coords.size(); ++i) {
    coords[i].x = data[2*i];
    coords[i].y = data[2*i + 1];
  }

  return coords;
}

Obstacle::ObstacleType Obstacle::type_get(void) const
{
  return _type;
}

ObstacleShape::ShapeType Obstacle::shape_get(void) const
{
  return _pimpl->shape_type();
}

std::vector<double> Obstacle::data_get(void) const
{
  return _pimpl->data();
}

void Obstacle::min_max_coordinates(double &min_x, double &max_x, double &min_y, double &max_y) const
{
  return _pimpl->min_max_coordinates(min_x, max_x, min_y, max_y);
}

bool Obstacle::is_inside(const Point<double> &pt) const
{
  return _pimpl->is_inside(pt);
}

bool Obstacle::on_border(const Point<double> &pt) const
{
  return _pimpl->on_border(pt);
}

std::pair<bool, double> Obstacle::ray_crossing(const Point<double> &pt1, const Point<double> &pt2) const
{
  return _pimpl->ray_crossing(pt1, pt2);
}

std::pair<double, std::vector<double>> Obstacle::distance_from_point(const Point<double> &pt) const
{
  return _pimpl->distance_from_point(pt);
}

void Obstacle::update_position(const Eigen::Vector2d &pos_delta)
{
  _pimpl->update_position(pos_delta);
}

} // namespace amrl