
#include <amrl_libs/display/TaskDisplay.hpp>

#include <amrl_display/AddPolygon.h>
#include <amrl_display/DeleteElement.h>

#include <nlohmann/json.hpp>
#include <fstream>

namespace amrl {

const std::string TaskDisplayManager::kDefaultColor("#c7065a");

TaskDisplayManager::TaskDisplayManager(ros::NodeHandle &nh, 
    const std::string &figure_label) : 
  _nh(nh),
  _fig_label(figure_label)
{
  _add_poly_client = _nh.serviceClient<amrl_display::AddPolygon>("/display/add_polygon");
  _del_client      = _nh.serviceClient<amrl_display::DeleteElement>("/display/delete_element");

  kCosAngles.resize(kNumSides);
  kSinAngles.resize(kNumSides);
  double dtheta = 2*M_PI / kNumSides;
  double offset = dtheta / 2.0;
  for (size_t i = 0; i < kNumSides; ++i) {
    double theta = i*dtheta + offset;
    kCosAngles[i] = cos(theta);
    kSinAngles[i] = sin(theta);
  }
}

void TaskDisplayManager::set_figure_label(const std::string &figure_label)
{
  _fig_label = figure_label;
}

void TaskDisplayManager::add_from_json_file(const std::string &json_task_file)
{
  using json = nlohmann::json;

  std::ifstream f(json_task_file, std::ifstream::in);
  if(f.is_open()) {
    json data = json::parse(f);
    if(!data.empty() && data.contains("tasks")) {
      json task_json = data["tasks"];

      for(json::iterator it = task_json.begin(); it != task_json.end(); ++it) {
        std::string task_name;
        std::string topic_name;
        Point<double> pos;
        double radius;
        std::string color;
        double alpha;


        if(it->contains("name")) {
          task_name = it->at("name");
        } else {
          // TODO:
        }

        if(it->contains("topic")) {
          topic_name = it->at("topic");
        } else {
          topic_name.clear();
        }

        if(it->contains("pos")) {
          pos.x = it->at("pos")[0];
          pos.y = it->at("pos")[1];
        }

        if(it->contains("color")) {
          color = it->at("color");
        } else {
          color = kDefaultColor;
        }

        if(it->contains("radius")) {
          radius = it->at("radius");
        } else {
          radius = kDefaultRadius;
        }

        if(it->contains("alpha")) {
          alpha = it->at("alpha");
        } else {
          alpha = kDefaultAlpha;
        }

        initialize_single(task_name, topic_name, pos, radius, color, alpha);
      }
    }
  } else {
    ROS_WARN("JSON file NOT open (likely invalid filename): %s", json_task_file.c_str()); 
  }
}

void TaskDisplayManager::initialize_single(
  const std::string &task_name,
  const std::string &topic_name,
  const Point<double> &pos,
  const double radius,
  const std::string &color,
  const double alpha)
{
  if(_task_names.find(task_name) == _task_names.end()) {

    if(_add_poly_client.waitForExistence(ros::Duration(2.0))) {
      amrl_display::AddPolygon srv;

      srv.request.header.fig_label = _fig_label;
      srv.request.header.name      = task_name;
      srv.request.header.topic     = topic_name;

      srv.request.config.alpha      = alpha;
      srv.request.config.color      = color;
      srv.request.config.fill       = true;
      srv.request.config.line_style = "";
      srv.request.config.line_width = 0.0;
      srv.request.config.zorder     = kZorder;
      
      srv.request.vertices = polygon_vertices(pos, radius);

      if(_add_poly_client.call(srv) && srv.response.result) {
        _task_names.insert(task_name);
        
        if(!topic_name.empty()) {
          _task_pubs[task_name] = std::pair<ros::Publisher, amrl_display::UpdatePolygon>();
          _task_pubs[task_name].first = _nh.advertise<amrl_display::UpdatePolygon>(topic_name, 1);

          _task_pubs[task_name].second.color    = color;
          _task_pubs[task_name].second.alpha    = alpha;
          _task_pubs[task_name].second.vertices = srv.request.vertices;
        }
      } else {
        ROS_WARN("Call to Add Task Display Service failed: %s", srv.response.message.c_str());
      }
    } else {
      ROS_WARN("Add task display client not available");
    }
  } else {
    ROS_WARN("Task '%s' has already been included.", task_name.c_str());
  }
}

void TaskDisplayManager::update_task(
  const std::string &rbt_name,
  const Point<double> &pos,
  const std::string &color,
  const double alpha)
{

}

std::vector<geometry_msgs::Point> TaskDisplayManager::polygon_vertices(const Point<double> &pos, double radius)
{
  std::vector<geometry_msgs::Point> vertices(kNumSides);

  for(size_t i = 0; i < kNumSides; ++i) {
    vertices[i].x = pos.x + radius * kCosAngles[i];
    vertices[i].y = pos.y + radius * kSinAngles[i];
  }

  return vertices;
}

} // namespace amrl