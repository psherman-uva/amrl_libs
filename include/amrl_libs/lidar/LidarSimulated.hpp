/*
  @file:      LidarSimulated.hpp
  @author:    psherman
  @date       July 2023
  
  @brief Class for simulating Lidar readings
*/

#pragma once

#include <amrl_libs/lidar/LidarSimulated.hpp>
#include <amrl_common/point/Point.hpp>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <cmath>

namespace amrl {

class LidarSimulated
{
public:

  /// Constructor
  /// @param range Maxmimum sensor range of each lidar beam
  /// @param num_beams Number of beams that make up full lidar sweep
  /// @param angle_sweep The total angle of the lidar field of view (centered at 0)
  LidarSimulated(const double range, const uint32_t num_beams, const double angle_sweep = 2*M_PI);

  /// Destructor (virtual for derived classes)
  virtual ~LidarSimulated(void) = default;

  /// Number of beams make up full lidar sweep
  /// @return number of lidar beams
  uint32_t num_beams(void) const;

  /// Get the lidar angle field of view
  /// @return Lidar angle field of view [rad]
  double angle_sweep(void) const;

  /// Get the current maximum range of the lidar sensor
  /// @return Current max range of sensor
  double range(void) const;

  /// @return Container of angles of each lidar beam
  const std::vector<double>& angles(void) const;

  /// Get a full measurement sweep from the lidar given a position in the environment
  /// @param pos Position in the world sensor is placed
  /// @param heading Rotation of the sensor
  /// @return Vector containing distance measuremet from each lidar beam
  virtual const std::vector<double>& measurement(const Point<double> &pos, const double heading) = 0;

  /// Check if the sensor thinks the position is inside an obstacle
  /// @param pos Position in the world of the sensor
  /// @return True if simualtion implementation thinks the lidar is inside an obstacle
  virtual bool position_in_obstacle(const Point<double> &pos) const = 0;

  /// Take a full measurment with the Lidar and (if active) publish
  /// @param pos Position in the world sensor is placed
  /// @param heading Rotation of the sensor
  /// @return Vector containing distance measuremet from each lidar beam
  const std::vector<double>& measurement_and_publish(const Point<double> &pos, const double heading);

  /// Dynamically change the maximum range of the lidar sensor
  /// @param range_new Updated max range of sensor to set
  void range_adjust(const double range_new);

  /// Enable ROS publishing for lidar measurments
  /// @param nh ROS node handle object
  /// @param topic Name of the topic to use when publishing scan messages
  /// @param frame_id Name of frame lidar is mapped to
  void setup_publishing(ros::NodeHandle &nh, const std::string &topic, const std::string &frame_id);

  /// Get access to the last updated scan from a lidar measurement
  /// @return The cached LaserScan message
  const sensor_msgs::LaserScan& get_last_scan(void) const;

  /// Get reference to container of the last lidar distance measurements
  /// @return Saved previous container of all range measurement
  const std::vector<double>& get_last_measurement(void) const;

  std::array<Point<double>, 2> field_of_view(const Point<double> &pos, const double heading) const;

protected: 

  /// Laser scan message output from a lidar measurement
  sensor_msgs::LaserScan _scan;

  /// Copy of distance measurements (converted to double to match most other things in package)
  std::vector<double> _dist_measurement;

  /// ROS publishder for laser scan
  ros::Publisher _pub;

  /// When true, ROS publisher is initialized and ready to publish
  bool _publishing;

  /// Angles of each lidar beam (in body frame)
  std::vector<double> kAngles;

  Point<double> _p1_fov;
  Point<double> _p2_fov;

  double _range;                 ///< Range of lidar sensing
  const uint32_t kNumBeams;      ///< Number of beams lidar uses
  const double kFieldOfView;     ///< Field of view (angle sweep) [radians]
  const double kThetaDelta;      ///< The angle difference [radians] between two beams
  float kDefaultMeasurementFlt;  ///< Distance measurement to use when beam doesn't hit an obstacle [float]
  double kDefaultMeasurementDbl; ///< Distance measurement to use when beam doesn't hit an obstacle [double]
};

}
