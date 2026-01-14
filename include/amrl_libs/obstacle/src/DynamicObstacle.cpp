
#include <amrl_libs/obstacle/DynamicObstacle.hpp>

#include <amrl_common/util/util.hpp>


namespace amrl {

DynamicObstacle::DynamicObstacle( const std::vector<Point<double>> &vertices, uint32_t id) : 
  Obstacle(ObstacleType::kDynamic, vertices),
  _anchor(Eigen::Vector2d::Zero()),
  _vel_cmd(Eigen::Vector2d::Zero()),
  _id(id)
{
  _anchor = vertices[0].to_eigen_vector();
}

DynamicObstacle::DynamicObstacle(const Point<double> &center, const double radius, uint32_t id) :
  Obstacle(ObstacleType::kDynamic, center, radius),
  _anchor(Eigen::Vector2d::Zero()),
  _vel_cmd(Eigen::Vector2d::Zero()),
  _id(id)
{
  _anchor[0] = center.x;
  _anchor[1] = center.y;
}

DynamicObstacle::DynamicObstacle(
    const Eigen::Vector2d &vel,
    const Point<double> &center,
    const double radius,
    uint32_t id) : 
  DynamicObstacle(center, radius, id)
{
  vel_set(vel);
}

Eigen::Vector2d DynamicObstacle::position_curr(void) const
{
  return _anchor;
}

void DynamicObstacle::position_set(const Point<double> &position)
{
  Eigen::Vector2d pos({position.x, position.y});
  Eigen::Vector2d delta = pos - _anchor; 
  update_position(delta);
  _anchor = pos;
}

void DynamicObstacle::drive(const double dt)
{
  Eigen::Vector2d delta = _vel_cmd * dt;
  _anchor += delta;
  update_position(delta);
}

void DynamicObstacle::vel_set(const Eigen::Vector2d &vel)
{
  _vel_cmd = vel;
}

Eigen::Vector2d DynamicObstacle::vel_curr(void) const
{
  return _vel_cmd;
}

void DynamicObstacle::id_set(uint32_t id)
{
  _id = id;
}

uint32_t DynamicObstacle::id_get(void) const
{
  return _id;
}

  
} // namespace amrl