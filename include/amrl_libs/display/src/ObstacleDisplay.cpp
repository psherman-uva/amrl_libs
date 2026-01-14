
#include <amrl_libs/display/ObstacleDisplay.hpp>
#include <amrl_display/DeleteElement.h>

namespace amrl {

const std::string ObstacleDisplayManager::kDefaultObsName  = "Obs_";
const std::string ObstacleDisplayManager::kDefaultDynName  = "DynObs_";
const std::string ObstacleDisplayManager::kDefaultMiscName = "MiscObs_";

std::map<Obstacle::ObstacleType, std::string> ObstacleDisplayManager::kColorMap({
  {Obstacle::ObstacleType::kSolid,     "#000000"},
  {Obstacle::ObstacleType::kDynamic,   "#047d0c"},
  {Obstacle::ObstacleType::kVibration, "#eb7d34"},
  {Obstacle::ObstacleType::kDoor,      "#0f42ba"}
});

ObstacleDisplayManager::ObstacleDisplayManager(ros::NodeHandle &nh, const std::string &figure_label)
  : _nh(nh),
    _node_name(ros::this_node::getName() + "/"),
    _fig_label(figure_label),
    _cnt_obs(0),
    _cnt_dyn(0),
    _cnt_misc(0)
{
  _add_poly_client   = _nh.serviceClient<amrl_display::AddPolygon>("/display/add_polygon");
  _add_circle_client = _nh.serviceClient<amrl_display::AddCircle>("/display/add_circle");
  _del_client        = _nh.serviceClient<amrl_display::DeleteElement>("/display/delete_element");
}

void ObstacleDisplayManager::set_figure_label(const std::string &figure_label)
{
  _fig_label = figure_label;
}

void ObstacleDisplayManager::add_from_json_file(const std::string &json_obs_file)
{
  std::vector<util::ObstacleSetupData> obs_data = util::obstacle_data_from_file(json_obs_file);
  add_from_setup_data(obs_data);
}

void ObstacleDisplayManager::add_from_setup_data(const std::vector<util::ObstacleSetupData> &setup_data) 
{
  std::vector<std::shared_ptr<Obstacle>> obs;
  for(const auto &o : setup_data) {
    std::shared_ptr<amrl::Obstacle> obs = util::obs_from_setup_data(o);
    
    std::string req_name;
    if(o.label.empty()){
      req_name = kDefaultObsName + std::to_string(_cnt_obs++);
    } else {
      req_name = o.label;
    }

    std::string color = o.color.empty() ? kColorMap.at(obs->type_get()) : o.color;
    double alpha      = o.alpha;

    add_obstacle(obs, req_name, color, alpha);
  }
}

void ObstacleDisplayManager::add_obstacle(std::shared_ptr<Obstacle> obs)
{
  std::string req_name = "";
  std::string color    = kColorMap.at(obs->type_get());
  double alpha         = 1.0;
  
  if(obs->type_get() == Obstacle::ObstacleType::kSolid) {
    req_name = kDefaultObsName + std::to_string(_cnt_obs++);
  } else if (obs->type_get() == Obstacle::ObstacleType::kDynamic) {
    req_name = kDefaultDynName + std::to_string(_cnt_dyn++);
  } else {
    req_name = kDefaultMiscName + std::to_string(_cnt_misc++);
  }

  add_obstacle(obs, req_name, color, alpha);
}


void ObstacleDisplayManager::add_obstacle(
  std::shared_ptr<Obstacle> obs,
  const std::string &obs_name,
  const std::string &color,
  const double alpha)
{
  if(_add_poly_client.waitForExistence(ros::Duration(5.0)) && 
      _add_circle_client.waitForExistence(ros::Duration(5.0))) {
    std::string topic = _node_name + obs_name;
    std::vector<double> obs_info = obs->data_get();
    
    if(obs->shape_get() == ObstacleShape::ShapeType::kCircle) {
      _circle_srv.request.header.fig_label = _fig_label;
      _circle_srv.request.header.name      = obs_name;
      _circle_srv.request.header.topic     = topic;
      
      _circle_srv.request.config.alpha = alpha;
      _circle_srv.request.config.color = color;
      _circle_srv.request.config.fill  = true;

      _circle_srv.request.center.x = obs_info[0];
      _circle_srv.request.center.y = obs_info[1];
      _circle_srv.request.radius   = obs_info[2];

      if(_add_circle_client.call(_circle_srv) && (!topic.empty())) {
        _circle_pub[obs_name] = std::pair<ros::Publisher, amrl_display::UpdateCircle>();
        _circle_pub[obs_name].first = _nh.advertise<amrl_display::UpdateCircle>(topic, 1);
        
        _circle_pub[obs_name].second.color  = _circle_srv.request.config.color;   
        _circle_pub[obs_name].second.center = _circle_srv.request.center;
        _circle_pub[obs_name].second.radius = _circle_srv.request.radius;
      } else {
        ROS_WARN("Call failed to add obstacle: %s", obs_name.c_str());
      }

    } else {
      size_t n = obs_info.size()/2;

      _poly_srv.request.header.fig_label = _fig_label;
      _poly_srv.request.header.name      = obs_name;
      _poly_srv.request.header.topic     = topic;
      
      _poly_srv.request.config.alpha = alpha;
      _poly_srv.request.config.color = color;
      
      
      _poly_srv.request.vertices.resize(n);
      for (size_t i = 0; i < n; ++i) {
        _poly_srv.request.vertices[i].x = obs_info[2*i];
        _poly_srv.request.vertices[i].y = obs_info[2*i + 1];
      }

      if(_add_poly_client.call(_poly_srv) && (!topic.empty())) {
        _poly_pub[obs_name] = std::pair<ros::Publisher, amrl_display::UpdatePolygon>();
        _poly_pub[obs_name].first = _nh.advertise<amrl_display::UpdatePolygon>(topic, 1);

        _poly_pub[obs_name].second.color    = _poly_srv.request.config.color;
        _poly_pub[obs_name].second.vertices = _poly_srv.request.vertices;
      } else {
        ROS_WARN("Call failed to obstacle: %s", obs_name.c_str());
      }
    }
  } else {
    ROS_WARN("Obstacle display service client not available.");
  }
}

void ObstacleDisplayManager::update_circle_obs(
  const std::string &obs_name,
  const Point<double> &pos,
  const std::string &color,
  const double alpha)
{
  if(_circle_pub.find(obs_name) != _circle_pub.end()) {
    _circle_pub[obs_name].second.color    = color;
    _circle_pub[obs_name].second.alpha    = alpha;
    _circle_pub[obs_name].second.center.x = pos.x;
    _circle_pub[obs_name].second.center.y = pos.y;
    
    _circle_pub[obs_name].first.publish(_circle_pub[obs_name].second);
  }
}

void ObstacleDisplayManager::update_polygon_obs(
  const std::string &obs_name,
  const std::vector<Point<double>> &vertices,
  const std::string &color,
  const double alpha)
{
if(_poly_pub.find(obs_name) != _poly_pub.end()) {
    _poly_pub[obs_name].second.color    = color;
    _poly_pub[obs_name].second.alpha    = alpha;

    _poly_pub[obs_name].second.vertices.resize(vertices.size());
    for(size_t i = 0; i < vertices.size(); ++i) {
      _poly_pub[obs_name].second.vertices[i].x = vertices[i].x;
      _poly_pub[obs_name].second.vertices[i].y = vertices[i].y;
    }
    
    _poly_pub[obs_name].first.publish(_poly_pub[obs_name].second);
  }
}

void ObstacleDisplayManager::remove_obs(const std::string &obs_name)
{
  amrl_display::DeleteElement del_srv;
  del_srv.request.name = obs_name;

  if(_del_client.call(del_srv)) {
    if(_circle_pub.find(obs_name) != _circle_pub.end()) {
      _circle_pub.erase(obs_name);
    }
    if(_poly_pub.find(obs_name) != _poly_pub.end()) {
      _poly_pub.erase(obs_name);
    }
  } else {
    ROS_WARN("Failed to delete obstacle: %s", obs_name.c_str());
  }
}

}