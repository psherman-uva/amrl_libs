
#include <amrl_libs/display/FigureDisplay.hpp>
#include <amrl_common/point/Point.hpp>

namespace amrl {

FigureDisplayManager::FigureDisplayManager(ros::NodeHandle &nh) :
  _nh(nh)
{
  _fig_client = _nh.serviceClient<amrl_display::CreateFigure>("/display/create_figure");
}

void FigureDisplayManager::add_from_ros_params(const std::string &ns)
{
  std::string fig_label = _nh.param<std::string>("/" + ns + "/label", "main");
  std::string fig_title = _nh.param<std::string>("/" + ns + "/title", "");
  std::string fig_color = _nh.param<std::string>("/" + ns + "/facecolor", "");
  double fig_size_X     = _nh.param<double>("/" + ns + "/size_x", 8.0);
  double fig_size_Y     = _nh.param<double>("/" + ns + "/size_y", 4.5);
  double fig_tick_size  = _nh.param<double>("/" + ns + "/tick_size", 5.0);
  bool fig_aspect_equal = _nh.param<bool>("/" + ns + "/aspect_equal", false);
  bool fig_show_grid    = _nh.param<bool>("/" + ns + "/show_grid", false);

  double kMapWidth      = _nh.param<double>("/map/width", 100);
  double kMapHeight     = _nh.param<double>("/map/height", 100);
  double kMapOriginX    = _nh.param<double>("/map/origin_x", 0.0);
  double kMapOriginY    = _nh.param<double>("/map/origin_y", 0.0);
  double kMapResolution = _nh.param<double>("/map/resolution", 0.1);
  Point<double> kMapOrigin(kMapOriginX, kMapOriginY);

  double kMapLimitX = kMapOriginX + kMapWidth; 
  double kMapLimitY = kMapOriginY + kMapHeight;

  if (_fig_client.waitForExistence(ros::Duration(2.0))) {
    amrl_display::CreateFigure fig_srv;

    fig_srv.request.title              = fig_title;
    fig_srv.request.label              = fig_label;
    fig_srv.request.facecolor          = fig_color;
    fig_srv.request.window_size_inches = std::vector<double>({fig_size_X, fig_size_Y});
    fig_srv.request.xy_limits          = std::vector<double>({kMapOriginX, kMapLimitX, kMapOriginY, kMapLimitY});
    fig_srv.request.tick_size          = fig_tick_size;
    fig_srv.request.x_label            = "$X$";
    fig_srv.request.y_label            = "$Y$";
    fig_srv.request.show_grid          = fig_show_grid;
    fig_srv.request.aspect_equal       = fig_aspect_equal;

    _fig_client.call(fig_srv);
  } else {
    ROS_WARN("Create Figure service not available");
  }
}

}