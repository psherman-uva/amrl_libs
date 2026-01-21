
#include <amrl_libs/obstacle/CircleObstacle.hpp>

#include <amrl_common/util/util.hpp>


namespace amrl {

CircleObstacle::CircleObstacle(const Point<double> &center, const double radius)
  : ObstacleShape(ShapeType::kCircle),
    _center(center),
    _radius(radius)
{
  _max_X = center.x + radius;
  _min_X = center.x - radius;
  _max_Y = center.y + radius;
  _min_Y = center.y - radius;

  _infinityX = _max_X + 1.0;
}

bool CircleObstacle::is_inside(const Point<double> &pt) const
{
  double dist = util::distance(pt, _center); // Distance from center
  return dist <= _radius;
}

bool CircleObstacle::on_border(const Point<double> &pt) const
{
  double dist = fabs(util::distance(pt, _center) - _radius); // Distance from border
  return dist <= kEps;
}

std::pair<double, std::vector<double>> CircleObstacle::distance_from_point(const Point<double> &pt) const
{
  std::pair<double, std::vector<double>> shortest_pt({0.0, std::vector<double>({0.0, 0.0})});

  double dist = util::distance(pt, _center);
  if (dist <= _radius) { return shortest_pt; }

  shortest_pt.first = dist - _radius;
  shortest_pt.second[0] = (_center.x - pt.x)/dist;
  shortest_pt.second[1] = (_center.y - pt.y)/dist;

  return shortest_pt;
}

std::pair<bool, double> CircleObstacle::ray_crossing(const Point<double> &pt1, const Point<double> &pt2) const
{  
  Point<double> p1 = pt1 - _center;
  Point<double> p2 = pt2 - _center;

  std::pair<bool, double> ray_cross({false, std::numeric_limits<double>::max()});

  Point<double> v = p2 - p1;
  double v_norm   = util::norm(v);
  double v_norm_2 = util::square(v_norm);
  
  double dot      = util::dot_product(p1, v);
  double dot_2    = util::square(dot);

  double delta    = 4*dot_2 - 4*v_norm_2*(util::square(util::norm(p1)) - util::square(_radius));
  if(std::abs(delta) < kEps) { delta = 0.0; } // Deal with floating point rounding errors

  if(delta >= 0.0) {
    double d  = sqrt(delta / 4.0);
    double t1 = (-dot + d)/v_norm_2;
    double t2 = (-dot - d)/v_norm_2;

    if((t1 >= 0.0 && t1 <= 1.0) || (t2 >= 0.0 && t2 <= 1.0)) {
      Point<double> x1 = p1 + t1 * v;
      Point<double> x2 = p1 + t2 * v;

      double dist1 = util::distance(p1, x1);
      double dist2 = util::distance(p1, x2);

      ray_cross.first  = true;
      ray_cross.second = std::min(dist1, dist2);
    }
  }

  return ray_cross;
}

std::vector<double> CircleObstacle::data(void) const
{
  std::vector<double> data(3);

  data[0] = _center.x;
  data[1] = _center.y;
  data[2] = _radius;

  return data;
}

void CircleObstacle::update_position(const Eigen::Vector2d &pos_delta)
{
  _center.x += pos_delta[0];
  _center.y += pos_delta[1];

  _max_X = _center.x + _radius;
  _min_X = _center.x - _radius;
  _max_Y = _center.y + _radius;
  _min_Y = _center.y - _radius;

  _infinityX = _max_X + 1.0;
}


} 