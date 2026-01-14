/*
  @file:      RobotData.hpp
  @author:    psherman
  @date       Sept 2024
*/

#pragma once

#include <amrl_libs/lidar/LidarSimulated.hpp>
#include <amrl_libs/mapping/OccupancyGrid.hpp>
#include <amrl_libs/display/OccupancyGridDisplay.hpp>
#include <amrl_libs/obstacle/Obstacle.hpp>
#include <amrl_libs/obstacle/DynamicObstacle.hpp>

#include <amrl_common/point/Point.hpp>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>

namespace amrl {

class RobotData
{
public:

  /// Constructor
  /// @param nh ROS Node handle object
  /// @param robot_label Name for robot
  /// @param ros_ns      Data of topic namespace
  RobotData(
    ros::NodeHandle &nh,
    const std::string &robot_label,
    const std::string &ros_ns = "");

  /// Default Destructor
  ~RobotData(void) = default;

  void cycle(void);

  void setup_vicon_pose_sub(ros::NodeHandle &nh, const std::string &vicon_obj);

  void setup_pose_pub(ros::NodeHandle &nh, const std::string &pub_topic);

  std::string label_get(void) const;
  
  std::shared_ptr<OccupancyGrid> map_get(void) const;
  std::shared_ptr<LidarSimulated> lidar_get(void) const;

  const std::vector<Obstacle>& obstacles_get(void) const;
  
  void dynamic_obstalce_add(std::shared_ptr<DynamicObstacle> dyn_obs);
  void dynamic_obstalce_remove(uint32_t obs_id);
  
  double theta_get(void) const;
  Point<double> pos_get(void) const;
  geometry_msgs::Pose pose_get(void) const;
  void pose_set(const geometry_msgs::Pose &pose);
  Eigen::Vector3d pose_vector_get(void) const;

protected:

  /// Common setup actions when constructing object
  /// @param nh 
  /// @param robot_label 
  /// @param ros_ns 
  void setup(
    ros::NodeHandle &nh,
    const std::string &robot_label,
    const std::string &ros_ns);

  /// Callback method to subscribe to robot position topic
  /// @param msg Position data published by topic
  void pose_callback(const geometry_msgs::Pose::ConstPtr &msg);

  /// Callback method to subscribe to vicon data for the pose
  /// @param msg Pose data published by vicon bridge
  void vicon_callback(const geometry_msgs::TransformStamped::ConstPtr &msg);

  // Name of Robot
  std::string _robot_label;

  //
  // Objects for Robot Stuff
  //

  /// Static obstacles in environment
  std::vector<Obstacle> _obstacles;

  /// Offset & Rotation [x, y, yaw] data from obstacles seen by Vicon vision capture system
  std::map<std::string, std::vector<double>> _vicon_obstacles;

  /// Dynamic obstalces in environment
  std::map<uint32_t, std::shared_ptr<DynamicObstacle>> _dyn_obstacles;

  /// Robots current idea of map
  std::shared_ptr<OccupancyGrid> _map;

  /// Publisher of map updates for visualization
  std::shared_ptr<OccupancyGridDisplay> _map_display;

  /// Simulated Lidar
  std::shared_ptr<LidarSimulated> _lidar;

  //
  // Robot State
  //

  /// Current Pose of robot
  geometry_msgs::Pose _pose;

  /// Robot XY position
  Point<double> _pos;

  /// Robot heading
  double _theta;

  /// Updated flag
  bool _pose_updated;

  //
  // ROS Specific stuff
  //

  /// Subscriber to robot pose
  ros::Subscriber _pose_sub;

  /// Publish pose in some situations
  ros::Publisher _pose_pub;
  bool _pose_is_published;

  /// Header frame identifier
  static const std::string kFrameId;
};



} // namespace amrl