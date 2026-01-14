
#include <amrl_libs/robot_data/RobotData.hpp>
#include <amrl_libs/lidar/LidarSimulatedMap.hpp>
#include <amrl_libs/lidar/LidarSimulatedObstacles.hpp>
#include <amrl_libs/obstacle/ObstacleHelpers.hpp>
#include <amrl_libs/util/ros_util.hpp>

#include <amrl_common/util/util.hpp>

#include <tf/tf.h>

namespace amrl {

RobotData::RobotData(
  ros::NodeHandle &nh,
  const std::string &robot_label,
  const std::string &ros_ns) :
    _pose_is_published(false)
{
  setup(nh, robot_label, ros_ns);

  // Robot pose subscription
  _pose_sub = nh.subscribe(robot_label + "/pose", 5, &RobotData::pose_callback, this);
}

void RobotData::setup_vicon_pose_sub(ros::NodeHandle &nh, const std::string &vicon_obj)
{
  _pose_sub.shutdown();
  _pose_sub = nh.subscribe("/vicon/" + vicon_obj + "/" + vicon_obj, 5, &RobotData::vicon_callback, this);
}

void RobotData::setup_pose_pub(ros::NodeHandle &nh, const std::string &pub_topic)
{
  _pose_is_published = true;
  _pose_pub = nh.advertise<geometry_msgs::Pose>(pub_topic, 5);
}

void RobotData::setup(
    ros::NodeHandle &nh,
    const std::string &robot_label,
    const std::string &ros_ns)
{
  const std::string frame_id   = "origin_frame";
  const std::string config_dir = nh.param<std::string>("/config_dir", "");
  const std::string obs_file   = nh.param<std::string>("/obs_file", "");
  const std::string vicon_file = nh.param<std::string>("/vicon_file", "");
  const bool is_experiment     = nh.param<bool>("/experiment", false);
  const bool is_lab            = nh.param<bool>("/in_lab", false);

  //
  // Occupancy Map & Display
  // 
  double kMapWidth      = nh.param<double>(ros_ns + "/map/width", 10);
  double kMapHeight     = nh.param<double>(ros_ns + "/map/height", 10);
  double kMapResolution = nh.param<double>(ros_ns + "/map/resolution", 0.1);
  float kMapOriginX     = nh.param<float>(ros_ns + "/map/origin_x", 0.0);
  float kMapOriginY     = nh.param<float>(ros_ns + "/map/origin_y", 0.0);
  GridInfo kMapInfo({kMapOriginX, kMapOriginY}, kMapWidth, kMapHeight, kMapResolution);

  _map         = std::make_shared<OccupancyGrid>(kMapInfo);
  _map_display = std::make_shared<OccupancyGridDisplay>(nh, robot_label, frame_id , kMapInfo);

  GridCallback display_cb = [this](uint32_t x, uint32_t y, double prob) { _map_display->update_data(x, y, prob); };
  _map->register_callback(display_cb);

  //
  // Setup Obstacles
  //
  if(!obs_file.empty()) {
    const std::string obs_file_full = config_dir + obs_file;
    _obstacles = util::solid_obstacles_from_file(obs_file_full);
  }
  
  if(!vicon_file.empty()) {
    const std::string vicon_file_full = config_dir + vicon_file;
    std::vector<Obstacle> vicon_obs = util::obstacles_with_vicon(nh, vicon_file_full);
    for(const auto &o : vicon_obs) {
      _obstacles.push_back(o);
    }
  }

  for (const auto &o : _obstacles) {
    std::vector<Point<uint32_t>> obs_cells = _map->cells_in_obstacle(o);
    for (uint8_t i = 0; i < 5; ++i) { _map->update_odds({}, obs_cells); }
  }

  //
  // Setup Lidar Simluation
  //
  double kLidarRange      = nh.param<double>(ros_ns + "/lidar/range", 3.0);
  double kLidarSweep      = nh.param<double>(ros_ns + "/lidar/sweep", 2*M_PI);
  int kLidarNumBeams      = nh.param<int>(ros_ns + "/lidar/num_beams", 360);
  bool LidarWithObstacles = nh.param<bool>(ros_ns + "/lidar/sim_with_obs", false); 

  if(LidarWithObstacles) {
    _lidar = std::make_shared<LidarSimulatedObstacles>(_obstacles, kLidarRange, kLidarNumBeams, kLidarSweep);
  } else {
    _lidar = std::make_shared<LidarSimulatedMap>(_map, kLidarRange, kLidarNumBeams, kLidarSweep);
  }
}

void RobotData::cycle(void)
{
  if(_pose_updated) {
    util::map_update_from_lidar_scan(_lidar, _map, _pos, _theta);
    _map_display->publish();
    _pose_updated = false;
  }

  if(_pose_is_published) {
    _pose_pub.publish(_pose);
  }
}

void RobotData::pose_callback(const geometry_msgs::Pose::ConstPtr &msg)
{
  _pose         = *msg;
  _pos.x        = msg->position.x;
  _pos.y        = msg->position.y;
  _theta        = tf::getYaw(msg->orientation);
  _pose_updated = true;
}

void RobotData::vicon_callback(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
  _pose.orientation = msg->transform.rotation;
  _pose.position.x  = msg->transform.translation.x;
  _pose.position.y  = msg->transform.translation.y;
  _pose.position.z  = msg->transform.translation.z;

  _pos.x = _pose.position.x;
  _pos.y = _pose.position.y;
  _theta = tf::getYaw(_pose.orientation);

  _pose_updated = true;
}

std::string RobotData::label_get(void) const
{
  return _robot_label;
}

const std::vector<Obstacle>& RobotData::obstacles_get(void) const
{
  return _obstacles;
}

void RobotData::dynamic_obstalce_add(std::shared_ptr<DynamicObstacle> dyn_obs)
{
  uint32_t id = dyn_obs->id_get();
  _dyn_obstacles[id] = dyn_obs;
}

void RobotData::dynamic_obstalce_remove(uint32_t obs_id)
{
  if(_dyn_obstacles.find(obs_id) != _dyn_obstacles.end()) {
    _dyn_obstacles.erase(obs_id);
  }
}

std::shared_ptr<OccupancyGrid> RobotData::map_get(void) const
{
  return _map;
}

std::shared_ptr<LidarSimulated> RobotData::lidar_get(void) const
{
  return _lidar;
}

geometry_msgs::Pose RobotData::pose_get(void) const
{
  return _pose;
}

Point<double> RobotData::pos_get(void) const
{
  return _pos;
}

Eigen::Vector3d RobotData::pose_vector_get(void) const
{
  Eigen::Vector3d x;
  x[0] = _pos.x;
  x[1] = _pos.y;
  x[2] = _theta;
  return x;
}

double RobotData::theta_get(void) const
{
  return _theta;
}

void RobotData::pose_set(const geometry_msgs::Pose &pose)
{
  _pose  = pose;
  _pos.x = _pose.position.x;
  _pos.y = _pose.position.y;
  _theta = tf::getYaw(_pose.orientation);
}

} // namespace amrl