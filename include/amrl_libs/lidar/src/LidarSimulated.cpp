#include <amrl_libs/lidar/LidarSimulated.hpp>

#include <amrl_common/util/util.hpp>
#include <Eigen/Dense>

namespace amrl {

LidarSimulated::LidarSimulated(
    const double range,
    const uint32_t num_beams,
    const double angle_sweep) 
: _publishing(false),
  _range(range),
  kNumBeams(num_beams),
  kFieldOfView(angle_sweep),
  kThetaDelta(angle_sweep/static_cast<double>(kNumBeams)),
  kDefaultMeasurementFlt(_range),
  kDefaultMeasurementDbl(_range)
{
  double half_sweep = angle_sweep / 2.0;
  for (size_t i = 0; i < kNumBeams; ++i) {
    double theta = i*kThetaDelta - half_sweep;
    kAngles.push_back(theta);
  }
  
  _dist_measurement.resize(kNumBeams);

  _scan.angle_min       = kAngles.front();
  _scan.angle_max       = kAngles.back();
  _scan.angle_increment = kThetaDelta;
  _scan.range_min       = 0.0;
  _scan.range_max       = _range*1.01;

  _scan.scan_time      = 0.1;
  _scan.time_increment = 0.1 / kNumBeams; 

  _scan.ranges.resize(kNumBeams);
  _scan.intensities.resize(kNumBeams, 47.0);

  _p1_fov.x = _range*cos(half_sweep);
  _p1_fov.x = _range*sin(half_sweep);
  _p2_fov.x = _range*cos(-half_sweep);
  _p2_fov.x = _range*sin(-half_sweep);
}

uint32_t LidarSimulated::num_beams(void) const
{
  return kNumBeams;
}

double LidarSimulated::angle_sweep(void) const
{
  return kFieldOfView;
}

double LidarSimulated::range(void) const
{
  return _range;
}

const std::vector<double>& LidarSimulated::angles(void) const
{
  return kAngles;
}

const std::vector<double>& LidarSimulated::measurement_and_publish(const Point<double> &pos, const double heading)
{
  measurement(pos, heading); // Measurement should update _scan ranges vector.
  ++_scan.header.seq;
  _scan.header.stamp = ros::Time::now();
  if(_publishing) { _pub.publish(_scan); }

  return _dist_measurement;
}

const sensor_msgs::LaserScan& LidarSimulated::get_last_scan(void) const
{
  return _scan;
}

const std::vector<double>& LidarSimulated::get_last_measurement(void) const
{
  return _dist_measurement;
}

void LidarSimulated::range_adjust(const double range_new)
{
  if(range_new > 0.0){
    _range                 = range_new;
    _scan.range_max        = _range * 1.01;
    kDefaultMeasurementFlt = _range;
    kDefaultMeasurementDbl = _range;
  }
}

void LidarSimulated::setup_publishing(ros::NodeHandle &nh, const std::string &topic, const std::string &frame_id)
{
  _scan.header.frame_id = frame_id;
  _scan.header.seq      = 0;
  _scan.header.stamp    = ros::Time::now();
  
  _pub = nh.advertise<sensor_msgs::LaserScan>(topic, 5);

  _publishing = true;
}

std::array<Point<double>, 2> LidarSimulated::field_of_view(const Point<double> &pos, const double heading) const
{
  // ct -st  x | x_b
  // st  ct  y | y_b
  //  0   0  1 | 1
  static Eigen::Matrix3d T(Eigen::Matrix3d::Identity());
  static Eigen::Vector3d p2_b(Eigen::Vector3d::Ones());
  static Eigen::Vector3d p1_b(Eigen::Vector3d::Ones());

  T(0, 0) =  cos(heading);
  T(0, 1) = -sin(heading);
  T(0, 2) =  pos.x;

  T(1, 0) =  sin(heading);
  T(1, 1) =  cos(heading);
  T(1, 2) =  pos.y;

  p1_b(0) = _p1_fov.x;
  p1_b(1) = _p1_fov.y;

  p2_b(0) = _p2_fov.x;
  p2_b(1) = _p2_fov.y;

  Eigen::Vector3d p1 = T * p1_b;
  Eigen::Vector3d p2 = T * p2_b;

  std::array<Point<double>, 2> p{{
    {p1(0), p1(1)}, 
    {p2(0), p2(1)} }};

  return p;
}


} // namespace amrl