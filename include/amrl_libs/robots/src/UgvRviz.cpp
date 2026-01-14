
#include <amrl_libs/robots/UgvRviz.hpp>
#include <nlohmann/json.hpp>

#include <amrl_display/AddRobot.h>

#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>

#include <fstream>

namespace amrl
{

UgvRviz::UgvRviz(ros::NodeHandle &nh, 
  const std::string &frame_id, 
  const std::string &marker_ns,
  const std::string &pub_namespace,
  const double scaling,
  const std::array<float, 3> &bdy_clr)
: _br()
{
  _body.header.frame_id = frame_id;
  _body.header.stamp    = ros::Time::now();
  _body.ns              = marker_ns;
  _body.id              = 0;
  _body.type            = visualization_msgs::Marker::CUBE;
  _body.action          = visualization_msgs::Marker::ADD;
  _body.lifetime        = ros::Duration();

  // Set the pose of the marker
  // Relative to the frame/time specified in the header
  _body.pose.position.x = 0.0;
  _body.pose.position.y = 0.0;
  _body.pose.position.z = scaling*0.1;
  _body.pose.orientation.x = 0.0;
  _body.pose.orientation.y = 0.0;
  _body.pose.orientation.z = 0.0;
  _body.pose.orientation.w = 1.0;

  // Scale size
  _body.scale.x = scaling*0.2;
  _body.scale.y = scaling*0.15;
  _body.scale.z = scaling*0.06;

  // Color
  _body.color.r = bdy_clr[0];
  _body.color.g = bdy_clr[1];
  _body.color.b = bdy_clr[2];
  _body.color.a = 1.0f;


  for(size_t i = 0; i < _wheels.size(); ++i) {
    tf2::Quaternion quat;
    quat.setRPY( M_PI/2, 0, 0 ); 

    _wheels[i].header.frame_id = frame_id;
    _wheels[i].header.stamp    = ros::Time::now();
    _wheels[i].ns              = marker_ns;
    _wheels[i].id              = 1 + i;
    _wheels[i].type            = visualization_msgs::Marker::CYLINDER;
    _wheels[i].action          = visualization_msgs::Marker::ADD;
    _wheels[i].lifetime        = ros::Duration();

    switch (i) {
      case 0:
        _wheels[i].pose.position.x = 0.062*scaling;
        _wheels[i].pose.position.y = 0.093*scaling;
        break;
      case 1:
        _wheels[i].pose.position.x = -0.062*scaling;
        _wheels[i].pose.position.y = 0.093*scaling;
        break;
      case 2:
        _wheels[i].pose.position.x = 0.062*scaling;
        _wheels[i].pose.position.y = -0.093*scaling;
        break;
      case 3:
        _wheels[i].pose.position.x = -0.062*scaling;
        _wheels[i].pose.position.y = -0.093*scaling;
        break;
    }
    _wheels[i].pose.position.z = 0.0425*scaling;
    _wheels[i].pose.orientation.x = quat.x();
    _wheels[i].pose.orientation.y = quat.y();
    _wheels[i].pose.orientation.z = quat.z();
    _wheels[i].pose.orientation.w = quat.w();

    _wheels[i].scale.x = 0.085*scaling;
    _wheels[i].scale.y = 0.085*scaling;
    _wheels[i].scale.z = 0.036*scaling;
    
    _wheels[i].color.r = 0.1f;
    _wheels[i].color.g = 0.1f;
    _wheels[i].color.b = 0.1f;
    _wheels[i].color.a = 1.0f;
  }

  _rviz_pub = nh.advertise<visualization_msgs::Marker>(pub_namespace + "/rosbot_markers", 5);
  
  // Transform Frame stuff
  static tf2::Quaternion quat;
  quat.setRPY(0, 0, 0);

  _transform_xy.header.frame_id         = "origin_frame";
  _transform_xy.child_frame_id          = frame_id + "_xy";
  _transform_xy.transform.translation.x = 0.0;
  _transform_xy.transform.translation.y = 0.0;
  _transform_xy.transform.translation.z = 0.0;
  _transform_xy.transform.rotation.x    = quat.x();
  _transform_xy.transform.rotation.y    = quat.y();
  _transform_xy.transform.rotation.z    = quat.z();
  _transform_xy.transform.rotation.w    = quat.w();


  _transform_theta.header.frame_id         = _transform_xy.child_frame_id;
  _transform_theta.child_frame_id          = frame_id;
  _transform_theta.transform.translation.x = 0.0;
  _transform_theta.transform.translation.y = 0.0;
  _transform_theta.transform.translation.z = 0.0;
  _transform_theta.transform.rotation.x    = quat.x();
  _transform_theta.transform.rotation.y    = quat.y();
  _transform_theta.transform.rotation.z    = quat.z();
  _transform_theta.transform.rotation.w    = quat.w();
}

void UgvRviz::update(const double x, const double y, const double theta)
{
  static tf2::Quaternion quat;
  
  if(isnan(x) || isnan(y) || isnan(theta)) {
    if(isnan(x)) {
      ROS_WARN("Ugv state has NaN: X");
    } 
    if (isnan(y)) {
      ROS_WARN("Ugv state has NaN: Y");
    }
    if (isnan(theta)) {
      ROS_WARN("Ugv state has NaN: Theta");
    }
    return;
  }

  _transform_xy.header.stamp = ros::Time::now();
  _transform_xy.transform.translation.x = x;
  _transform_xy.transform.translation.y = y;

  quat.setRPY(0, 0, theta);
  _transform_theta.header.stamp = ros::Time::now();
  _transform_theta.transform.rotation.x = quat.x();
  _transform_theta.transform.rotation.y = quat.y();
  _transform_theta.transform.rotation.z = quat.z();
  _transform_theta.transform.rotation.w = quat.w();

  _br.sendTransform(_transform_xy);
  _br.sendTransform(_transform_theta);

  auto ts = ros::Time::now();
  _body.header.stamp = ts;
  _rviz_pub.publish(_body);
  
  for(auto &w : _wheels) {
    w.header.stamp = ts;
    _rviz_pub.publish(w);
  }
}

} // namespace amrl