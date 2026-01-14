
#include <amrl_libs/util/ros_util.hpp>
#include <amrl_libs/obstacle/ObstacleHelpers.hpp>
#include <amrl_common/util/util.hpp>
#include <amrl_common/util/bresenham_line.hpp>

#include <logging_util/util.hpp>
#include <amrl_logging/LoggingData.h>

#include <ros/master.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include <nlohmann/json.hpp>
#include <fstream>

namespace amrl {
namespace util {

double vel_twist_norm(const geometry_msgs::Twist &vel)
{
  // double vl = fabs(vel.linear.x - 0.5*vel.angular.z);
  // double vr = fabs(vel.linear.x + 0.5*vel.angular.z);
  // return (vl + vr) / 2.0;

  return vel.linear.x;
}

void reset_pose(ros::NodeHandle &nh, const std::string &robot_namespace) 
{
  /// Sending an empty message to the set_pose topic will reinitialize the pose topic.
  std::string reset_topic = "/" + robot_namespace + "/set_pose";
  ros::Publisher pub      = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(reset_topic, 1, true);
  
  // Send pose of zeros
  geometry_msgs::PoseWithCovarianceStamped msg;
  pub.publish(msg);
  
  // For some reason, seems like the message
  // needs to be published a few times or it
  // won't actually take effect. ¯\_(ツ)_/¯  
  //   --pdsherman
  ros::Rate rate(10);
  for(int i = 0; i < 10; ++i) {
    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }
}

geometry_msgs::Quaternion quaternion_from_yaw(const double yaw)
{
  static tf2::Quaternion quat;
  static geometry_msgs::Quaternion q;
  
  quat.setRPY(0, 0, yaw);

  q.x = quat.x();
  q.y = quat.y();
  q.z = quat.z();
  q.w = quat.w();

  return q;
}

bool obstacle_lidar_in_view( 
  std::shared_ptr<LidarSimulated> lidar,
  std::shared_ptr<Obstacle> obs,
  const Point<double> &pos,
  const double &theta)
{
  const std::vector<double>& angles = lidar->angles();
  const std::vector<double>& Z      = lidar->measurement(pos, theta);

  for (uint32_t i = 0; i < Z.size(); ++i) {
    double lidar_theta = angles[i]; 
    double ray_theta   = theta + lidar_theta;
    double dist        = Z[i];

    Point<double> d_pos = Point<double>(cos(ray_theta), sin(ray_theta)) * dist;
    Point<double> pos2  = pos + d_pos;

    std::pair<bool, double> ray_crossing = obs->ray_crossing(pos, pos2);
    if(ray_crossing.first) { return true; }
  }

  return false;
}

std::vector<std::vector<uint32_t>> lidar_perceptual_range(
  std::shared_ptr<LidarSimulated> lidar,
  std::shared_ptr<OccupancyGrid> map,
  const Point<double> &pos,
  const double heading)
{
  std::vector<std::vector<uint32_t>> cells_idx;

  Point<int32_t> p1                 = map->point_to_cell(pos);
  const std::vector<double>& angles = lidar->angles();
  double range                      = lidar->range();

  for (const auto &lidar_theta : angles) {
    double theta = heading + lidar_theta;
    Point<double> d_pos = Point<double>(cos(theta), sin(theta)) * range;
    Point<double> pos2  = pos + d_pos;
    Point<int32_t> p2  = map->point_to_cell(pos2);
    std::vector<Point<int32_t>> pts = util::bresenham_line(p1, p2);

    std::vector<uint32_t> indices;
    for (const auto &pt : pts) {
      if(map->cell_in_grid(pt)) {
        uint32_t idx = map->cell_to_index(pt);
        indices.push_back(idx);
      }
    }
    cells_idx.push_back(indices);
  }

  return cells_idx;
}

std::set<uint32_t> lidar_cells_full_range( 
  std::shared_ptr<LidarSimulated> lidar,
  std::shared_ptr<OccupancyGrid> map,
  const Point<double> &pos,
  const double &theta)
{
  std::set<uint32_t> cells_idx;

  Point<int32_t> p1                 = map->point_to_cell(pos);
  const std::vector<double>& angles = lidar->angles();
  const double range                = lidar->range();

  for (const auto &lidar_theta : angles) {
    double theta_0      = theta + lidar_theta;
    Point<double> d_pos = Point<double>(range*cos(theta_0), range*sin(theta_0));
    Point<double> pos2  = pos + d_pos;
    Point<int32_t> p2   = map->point_to_cell(pos2);

    std::vector<Point<int32_t>> pts = util::bresenham_line(p1, p2);
    for (const auto &pt : pts) {
      if(map->cell_in_grid(pt)) {
        cells_idx.insert(map->cell_to_index(pt));
      }
    }
  }

  return cells_idx;
}

std::vector<std::vector<uint32_t>> lidar_measurement_range(
  std::shared_ptr<LidarSimulated> lidar,
  std::shared_ptr<OccupancyGrid> map,
  const Point<double> &pos,
  const double heading)
{
  std::vector<std::vector<uint32_t>> cells_idx;
  
  Point<int32_t> p1                 = map->point_to_cell(pos);
  const std::vector<double>& angles = lidar->angles();
  const std::vector<double>& Z      = lidar->measurement(pos, heading);

  for (uint32_t i = 0; i < Z.size(); ++i) {
    double theta = heading + angles[i];
    double dist  = Z[i];

    Point<double> d_pos = Point<double>(cos(theta), sin(theta)) * dist;
    Point<double> pos2  = pos + d_pos;
    Point<int32_t> p2  = map->point_to_cell(pos2);
    std::vector<Point<int32_t>> pts = util::bresenham_line(p1, p2);

    std::vector<uint32_t> indices;
    for (const auto &pt : pts) {
      if(map->cell_in_grid(pt)) {
        uint32_t idx = map->cell_to_index(pt);
        indices.push_back(idx);
      }
    }
    cells_idx.push_back(indices);
  }

  return cells_idx;
}

std::set<uint32_t> lidar_cells_seen( 
  std::shared_ptr<LidarSimulated> lidar,
  std::shared_ptr<OccupancyGrid> map,
  const Point<double> &pos,
  const double &theta)
{
  std::set<uint32_t> cells_idx;
  
  Point<int32_t> p1                 = map->point_to_cell(pos);
  const std::vector<double>& angles = lidar->angles();
  const std::vector<double>& Z      = lidar->measurement(pos, theta);

  for (uint32_t i = 0; i < Z.size(); ++i) {
    double angle = theta + angles[i];
    double dist  = Z[i];

    Point<double> d_pos = Point<double>(cos(angle), sin(angle)) * dist;
    Point<double> pos2  = pos + d_pos;
    Point<int32_t> p2   = map->point_to_cell(pos2);
    std::vector<Point<int32_t>> pts = util::bresenham_line(p1, p2);

    for (const auto &pt : pts) {
      if(map->cell_in_grid(pt)) {
        cells_idx.insert(map->cell_to_index(pt));
      }
    }
  }

  return cells_idx;
}

std::set<uint32_t> lidar_cells_blocked( 
  std::shared_ptr<LidarSimulated> lidar,
  std::shared_ptr<OccupancyGrid> map,
  const Point<double> &pos,
  const double &heading)
{
  std::set<uint32_t> blocked;

  const std::vector<double>& angles = lidar->angles();
  const std::vector<double>& Z      = lidar->measurement(pos, heading);
  double range                      = lidar->range();

  for (uint32_t i = 0; i < Z.size(); ++i) {
    double theta = heading + angles[i];
    double dist  = Z[i];

    Point<double> dPos = Point<double>(cos(theta), sin(theta));
    if(dist < range) { dist += map->resolution() / 4.0; }
    Point<double> p1 = pos + dPos * dist;
    Point<double> p2 = pos + dPos * range;

    Point<int32_t> p1_int  = map->point_to_cell(p1);
    Point<int32_t> p2_int  = map->point_to_cell(p2);
    std::vector<Point<int32_t>> pts = util::bresenham_line(p1_int, p2_int);

    if(pts.size() > 1) {
      for (auto it = pts.begin()+1; it != pts.end(); ++it) {
        if(map->cell_in_grid(*it)) {
          blocked.insert(map->cell_to_index(*it));
        }
      }
    }
  }

  return blocked;
}

void map_update_from_lidar_scan(
  std::shared_ptr<LidarSimulated> lidar,
  std::shared_ptr<OccupancyGrid> map,
  const Point<double> &rbt_pos,
  const double rbt_heading)
{
  const std::vector<double>& measurement = lidar->measurement(rbt_pos, rbt_heading);
  const std::vector<double>& angles      = lidar->angles();

  for (size_t i = 0; i < angles.size(); ++i) {
    double dist = measurement.at(i);
    if(dist > 0.0) {
      double theta = rbt_heading + angles.at(i);
      map_update_from_lidar_ray(lidar, map, rbt_pos, theta, dist);
    }
  }
}

void map_update_from_lidar_ray(
  std::shared_ptr<LidarSimulated> lidar,
  std::shared_ptr<OccupancyGrid> map,
  const Point<double> &rbt_pos,
  const double theta,
  const double dist)
{
  static const std::vector<Point<int32_t>> kNeighbors({
      {-1, -1}, { 0, -1}, 
      { 1, -1}, {-1,  0}, 
      { 1,  0}, {-1,  1}, 
      { 0,  1}, { 1,  1}});

  bool end_point_is_obs = (dist != lidar->range());

  // Get starting cell from robot position
  Point<uint32_t> start = map->point_to_cell(rbt_pos);

  // Get ending cell using start, theta, distance
  Point<double> pt_end(rbt_pos.x + dist*cos(theta), rbt_pos.y + dist*sin(theta));
  Point<uint32_t> end = map->point_to_cell(pt_end);

  // Line of grid points connecting start to end
  Point<int32_t> p1(start.x, start.y);
  Point<int32_t> p2(end.x, end.y);
  auto pts = util::bresenham_line(p1, p2);

  // Populate containers with cells measured free or occupied
  std::vector<Point<uint32_t>> free_cells;
  std::vector<Point<uint32_t>> occ_cells;
  std::vector<Point<uint32_t>> neighbor_cells;

  if (pts.size() > 1) {
    std::for_each(pts.begin(), pts.end()-2, [&free_cells](const Point<int32_t> &pt) {
      free_cells.emplace_back(pt.x, pt.y);
    });

    Point<uint32_t> p_back(pts.back().x, pts.back().y); 
    if(end_point_is_obs) {
      occ_cells.emplace_back(pts.back().x, pts.back().y);
      // for(const auto &n : kNeighbors) {
      //   neighbor_cells.emplace_back(p_back.x + n.x, p_back.y + n.y);
      // }
    } else {
      free_cells.emplace_back(pts.back().x, pts.back().y);

      Point<int32_t>  p2s = *(pts.rbegin()+1);
      free_cells.emplace_back(p2s.x, p2s.y);
    }
  } else {
    occ_cells.emplace_back(pts.back().x, pts.back().y);
  }


  // Update grid
  map->update_odds(free_cells, occ_cells);
}

std::vector<Obstacle> map_setup(ros::NodeHandle &nh, 
  const std::string &obs_file, 
  const std::string &frame_id, 
  std::shared_ptr<OccupancyGrid> map)
{
  if (obs_file.empty()) { return std::vector<Obstacle>(); }

  auto obs = util::solid_obstacles_from_file(obs_file);
  for (const auto &o : obs) {
    std::vector<Point<uint32_t>> obs_cells = map->cells_in_obstacle(o);
    for (uint8_t i = 0; i < 5; ++i) { map->update_odds({}, obs_cells); }
  }

  return obs;
}

bool logging_full_setup(
  ros::NodeHandle &nh,
  ros::Publisher &data_pub,
  amrl_logging::LoggingData &data,
  const std::string &table_name,
  const std::string &topic_name,
  const std::vector<std::string> &labels_header,
  const std::vector<std::string> &ints_header,
  const std::vector<std::string> &reals_header,
  bool delete_table)
{
  const std::string kDataTopic = ros::this_node::getName() + "/" + topic_name;

  if(delete_table && !amrl::logging_delete_table(nh, table_name)) { return false; }
  if(!amrl::logging_setup(nh, table_name, kDataTopic, labels_header, ints_header, reals_header)) { return false; }

  data_pub = nh.advertise<amrl_logging::LoggingData>(kDataTopic, 50);

  data.labels.resize(labels_header.size());
  data.nums.resize(ints_header.size());
  data.reals.resize(reals_header.size());

  for (int i = 0; i < 15; ++i) { ros::Rate(10).sleep(); } // Takes a second for logging to get started.

  return true;
}


std::vector<std::string> robot_names_from_file(const std::string &full_rbt_file)
{
  std::vector<std::string> rbt_names;
  
  if(!full_rbt_file.empty()) {
    ROS_INFO("Getting Robots from file: %s", full_rbt_file.c_str());

    std::ifstream f(full_rbt_file);
    if(f.is_open()) {
      nlohmann::json data = nlohmann::json::parse(f)["robots"];
      for(nlohmann::json::iterator it = data.begin(); it != data.end(); ++it) {
        std::string robot_name = it.value()["name"];
        rbt_names.push_back(robot_name);
      }
    } else {
      ROS_WARN("Failed to open Robot file: %s", full_rbt_file.c_str());
    }
  } else {
    ROS_WARN("Robot file empty: %s", full_rbt_file.c_str());
  }
  return rbt_names;
}

} // namespace util
} // namespace amrl
