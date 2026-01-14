
#include <amrl_libs/obstacle/PolygonObstacle.hpp>

#include <amrl_common/util/util.hpp>

#include <limits>


namespace amrl {

PolygonObstacle::PolygonObstacle(const std::vector<Point<double>> &coordinates)
  : ObstacleShape(ShapeType::kPolygon),
    _coordinates(coordinates),
    kNumCoords(coordinates.size())
{
  set_min_max_values();
}

std::vector<double> PolygonObstacle::data(void) const
{
  std::vector<double> data(kNumCoords * 2);

  for(size_t i = 0; i < kNumCoords; ++i) {
    data[2*i]     = _coordinates[i].x;
    data[2*i + 1] = _coordinates[i].y;
  }

  return data;
}

bool PolygonObstacle::is_inside(const Point<double> &pt) const
{
  /// Implements a ray casting alogorithm to determine if p is inside the obstacle.
  /// Idea is to start at the point, then cast a line horizontaly
  /// to "infinity" on the right. Couning the number of times the 
  /// ray crosses a polygon boundary determines if the point is inside

  bool is_inside = false;

  // First, check if point is inside the biggest bounding box possible.
  // If so, then do full ray-cast algorithm
  if (pt.x <= _max_X && pt.x >= _min_X && pt.y <= _max_Y && pt.y >= _min_Y) {

    // Special case 1: Point IS a vertex of polygon
    for(const auto &coord : _coordinates){
      if (pt.x == coord.x && pt.y == coord.y) { return true; }
    }

    // Point to the right of all points in polygon
    Point<double> pt_inf(_infinityX, pt.y);

    size_t cnt = 0; // Count number of intersections
    for(size_t i = 0; i < kNumCoords; ++i) {
      size_t next_i = (i+1) % kNumCoords;

      // Special case 2: Point is on a line between two vertices
      if(orientation(_coordinates[i], _coordinates[next_i], pt) == 0 &&
          on_segment(_coordinates[i], _coordinates[next_i], pt)) {
        return true;
      }

      // Nominal Case: Check if casted ray intersects the line created by vertex i and "next" coordinate
      if (do_intersect(pt, pt_inf, _coordinates[i], _coordinates[next_i])) {
        if (pt.y == _coordinates[next_i].y) {
          // Need to consider edge cases when ray goes right through a vertex.
          // Decrementing count depeneds on if:
          //    - vertex is a top/bottom point or side 
          //    - side is exactly horizontal
          size_t next_next = (next_i+1) % kNumCoords;
          if((_coordinates[i].y < pt.y) != (_coordinates[next_next].y < pt.y) || 
              (_coordinates[i].y == _coordinates[next_i].y)  ) {
            --cnt;
          }
        }
        ++cnt;
      }
    }

    is_inside = (cnt%2) != 0;
  }

  return is_inside;
}

bool PolygonObstacle::on_border(const Point<double> &pt) const
{
  // Case 1: Point IS a vertex
  for(const auto &coord : _coordinates){
    double dist = util::distance(pt, coord);
    if (dist <= kEps) { return true; }
  }

  // Case 2: Point is on line between vertices
  for(size_t i = 0; i < kNumCoords; ++i) {
    size_t next_i = (i+1) % kNumCoords;\
    if(orientation(_coordinates[i], _coordinates[next_i], pt) == 0 &&
        on_segment(_coordinates[i], _coordinates[next_i], pt)) {
      return true;
    }
  }

  return false;
}

std::pair<double, std::vector<double>> PolygonObstacle::distance_from_point(const Point<double> &pt) const
{
  std::pair<double, std::vector<double>> shortest_pt({0.0, std::vector<double>({0.0, 0.0})});

  if (is_inside(pt)) { return shortest_pt; }

  Eigen::Vector2f v1;
  Eigen::Vector2f v2;
  Eigen::Vector2f p({pt.x, pt.y});

  double dist = std::numeric_limits<double>::max();
  double dist_temp = 0.0;
  std::vector<double> dist_dir(2);

  for(size_t i = 0; i < kNumCoords; ++i) {
    size_t next = (i+1) % kNumCoords;
    v1[0] = _coordinates[i].x;
    v1[1] = _coordinates[i].y;
    v2[0] = _coordinates[next].x;
    v2[1] = _coordinates[next].y;
    Eigen::Vector2f v  = v2 - v1;
    Eigen::Vector2f p2 = ((v*v.transpose())/(v.transpose()*v))*(p - v1) + v1;

    Point<double> pt2({p2[0], p2[1]});

    if (do_intersect(pt, pt2, _coordinates[i], _coordinates[next])) {
      dist_temp  = distance_to_intersection(pt, pt2, _coordinates[i], _coordinates[next]);

      dist_dir[0] = pt2.x - pt.x;
      dist_dir[1] = pt2.y - pt.y;
    } else {
      double d1 = util::distance(pt, _coordinates[i]);
      double d2 = util::distance(pt, _coordinates[next]);
      if (d1 < d2) {
        dist_temp = d1;
        dist_dir[0] = _coordinates[i].x - pt.x;
        dist_dir[1] = _coordinates[i].y - pt.y;
      } else {
        dist_temp = d2;
        dist_dir[0] = _coordinates[next].x - pt.x;
        dist_dir[1] = _coordinates[next].y - pt.y;
      }
    }

    if(dist_temp < dist) {
      dist = dist_temp;

      shortest_pt.first  = dist;
      shortest_pt.second = dist_dir;
    }
  }

  shortest_pt.second[0] /= dist;
  shortest_pt.second[1] /= dist;

  return shortest_pt;
}

std::pair<bool, double> PolygonObstacle::ray_crossing(const Point<double> &p1, const Point<double> &p2) const
{
  // Check every ordered pair of coordinates. Save closest distance
  std::pair<bool, double> ray_cross({false, std::numeric_limits<double>::max()});

  for(size_t i = 0; i < kNumCoords; ++i) {
    size_t next = ((i+1) % kNumCoords);

    if (do_intersect(p1, p2, _coordinates[i], _coordinates[next])) {
      double dist_check  = distance_to_intersection(p1, p2, _coordinates[i], _coordinates[next]);
      
      if (dist_check < ray_cross.second) {
        ray_cross.first  = true;
        ray_cross.second = dist_check;
      }

    }
  }

  return ray_cross;  
}

void PolygonObstacle::update_position(const Eigen::Vector2d &pos_delta)
{
  for(auto &p : _coordinates) {
    p.x += pos_delta[0];
    p.y += pos_delta[1];
  }
  set_min_max_values();
}

bool PolygonObstacle::do_intersect(
  const Point<double> &p11,
  const Point<double> &p12,
  const Point<double> &p21,
  const Point<double> &p22) const
{
  int o1 = orientation(p11, p12, p21);
  int o2 = orientation(p11, p12, p22);
  int o3 = orientation(p21, p22, p11);
  int o4 = orientation(p21, p22, p12);

  if ((o1 != o2) && (o3 != o4))
    return true;
  else if (o1 == 0 && on_segment(p11, p12, p21))
    return true;
  else if (o2 == 0 && on_segment(p11, p12, p22))
    return true;
  else if (o3 == 0 && on_segment(p21, p22, p11))
    return true;
  else if (o4 == 0 && on_segment(p21, p22, p12))
    return true;
  
  return false;
}

int PolygonObstacle::orientation(const Point<double> &p1, const Point<double> &p2, const Point<double> &p3) const
{  
  double val = (p2.y-p1.y)*(p3.x-p2.x) - (p2.x-p1.x)*(p3.y-p2.y);
  if (std::fabs(val) < kEps)
    return 0;
  else if (val > 0)
    return 1;
  else
    return -1;
}

bool PolygonObstacle::on_segment(const Point<double> &p1, const Point<double> &p2, const Point<double> &p) const
{
  // If vertical line, check if point is between the min/max 'y' values
  // Otherwise, check if between min/max 'x'
  if(std::fabs(p1.x - p2.x) < kEps) {
    return  p.y <= std::max(p1.y, p2.y) && p.y >= std::min(p1.y, p2.y);
  }
  return p.x <= std::max(p1.x, p2.x) && p.x >= std::min(p1.x, p2.x);
}

double PolygonObstacle::distance_to_intersection(
    const Point<double> &p1, 
    const Point<double> &p2, 
    const Point<double> &coord1, 
    const Point<double> &coord2) const
{
  //
  // Algorithm found at following link to get point of intersection:
  //     https://www.geeksforgeeks.org/program-for-point-of-intersection-of-two-lines/
  //

  double a1 = p2.y - p1.y;
  double b1 = p1.x - p2.x;
  double c1 = a1*p1.x + b1*p1.y;

  double a2 = coord2.y - coord1.y;
  double b2 = coord1.x - coord2.x;
  double c2 = a2*coord1.x + b2*coord1.y;

  double determinant = a1*b2 - a2*b1;

  if (fabs(determinant) < kEps) {
    return std::min<double>(util::distance(p1, coord1), util::distance(p1, coord2));
  }

  Point<double> p_intersect((b2*c1 - b1*c2)/determinant, (a1*c2 - a2*c1)/determinant);
  return util::distance(p1, p_intersect);
}

void PolygonObstacle::set_min_max_values(void)
{
  _max_X = std::numeric_limits<double>::lowest();
  _min_X = std::numeric_limits<double>::max();
  _max_Y = std::numeric_limits<double>::lowest();
  _min_Y = std::numeric_limits<double>::max();

  for(const auto& elem : _coordinates) {
    if(elem.x > _max_X) { _max_X = elem.x; };
    if(elem.x < _min_X) { _min_X = elem.x; };
    if(elem.y > _max_Y) { _max_Y = elem.y; };
    if(elem.y < _min_Y) { _min_Y = elem.y; };
  }

  // Adding some arbitraty amount to maximum x position
  _infinityX = _max_X + 1.0;
}

}
