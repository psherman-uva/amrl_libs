
#include <amrl_libs/display/FigureDisplay.hpp>
#include <amrl_common/point/Point.hpp>

namespace amrl {

FigureDisplayManager::FigureDisplayManager(ros::NodeHandle &nh) :
  _nh(nh)
{
  _fig_client = _nh.serviceClient<amrl_display::CreateFigure>("/display/create_figure");
}

void FigureDisplayManager::add_from_ros_params(
  const std::string &ns,
  bool use_map_limits)
{
  std::string fig_label   = _nh.param<std::string>("/" + ns + "/label", "");
  std::string fig_title   = _nh.param<std::string>("/" + ns + "/title", "");
  std::string fig_color   = _nh.param<std::string>("/" + ns + "/facecolor", "");
  std::string fig_x_label = _nh.param<std::string>("/" + ns + "/x_label", "$X$");
  std::string fig_y_label = _nh.param<std::string>("/" + ns + "/y_label", "$Y$");

  double fig_size_X     = _nh.param<double>("/" + ns + "/size_x", 8.0);
  double fig_size_Y     = _nh.param<double>("/" + ns + "/size_y", 4.5);
  double fig_tick_size  = _nh.param<double>("/" + ns + "/tick_size", 5.0);

  bool fig_y_rotate     = _nh.param<bool>("/" + ns + "/rotate_y", false);
  bool fig_aspect_equal = _nh.param<bool>("/" + ns + "/aspect_equal", false);
  bool fig_show_grid    = _nh.param<bool>("/" + ns + "/show_grid", false);

  std::vector<double> xy_limits;
  if(use_map_limits) {
    double kMapWidth      = _nh.param<double>("/map/width", 100);
    double kMapHeight     = _nh.param<double>("/map/height", 100);
    double kMapOriginX    = _nh.param<double>("/map/origin_x", 0.0);
    double kMapOriginY    = _nh.param<double>("/map/origin_y", 0.0);

    xy_limits = std::vector<double>({kMapOriginX, kMapOriginX + kMapWidth, kMapOriginY, kMapOriginY + kMapHeight});
  } else {
    double min_x, max_x, min_y, max_y;
    if(_nh.getParam("/" + ns + "/min_x", min_x) && _nh.getParam("/" + ns + "/max_x", max_x) &&
      _nh.getParam("/" + ns + "/min_y", min_y) && _nh.getParam("/" + ns + "/max_y", max_y) )
    {
      xy_limits = std::vector<double>({min_x, max_x, min_y, max_y});
    }
  }

  if (_fig_client.waitForExistence(ros::Duration(2.0))) {
    amrl_display::CreateFigure fig_srv;

    fig_srv.request.title              = fig_title;
    fig_srv.request.label              = fig_label;
    fig_srv.request.facecolor          = fig_color;
    fig_srv.request.window_size_inches = std::vector<double>({fig_size_X, fig_size_Y});
    fig_srv.request.xy_limits          = xy_limits;
    fig_srv.request.tick_size          = fig_tick_size;
    fig_srv.request.x_label            = fig_x_label;
    fig_srv.request.y_label            = fig_y_label;
    fig_srv.request.rotate_y           = fig_y_rotate;
    fig_srv.request.show_grid          = fig_show_grid;
    fig_srv.request.aspect_equal       = fig_aspect_equal;

    _fig_client.call(fig_srv);
  } else {
    ROS_WARN("Create Figure service not available");
  }
}

}