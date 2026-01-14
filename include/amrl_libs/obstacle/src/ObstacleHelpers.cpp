
#include <amrl_libs/obstacle/ObstacleHelpers.hpp>
#include <amrl_libs/obstacle/DynamicObstacle.hpp>
#include <amrl_libs/obstacle/VibrationObstacle.hpp>

#include <nlohmann/json.hpp>

#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <fstream>

namespace amrl {
namespace util {

std::vector<ObstacleSetupData> obs_data_from_json_obj(nlohmann::json &obs_json, const Obstacle::ObstacleType obs_type);

void obstacleCallback(const geometry_msgs::TransformStamped::ConstPtr &msg, ObstacleSetupData &obs_data)
{
  obs_data.pos_set = true;
  obs_data.x       = msg->transform.translation.x;
  obs_data.y       = msg->transform.translation.y;
  obs_data.theta   = tf::getYaw(msg->transform.rotation);
}

std::shared_ptr<Obstacle> obs_from_setup_data(const ObstacleSetupData &obs_data)
{
  if(obs_data.shape == ObstacleShape::ShapeType::kPolygon) {
    std::vector<Point<double>> vertices = obs_data.vertices;
    
    double ct = cos(obs_data.theta);
    double st = sin(obs_data.theta);
    double x  = obs_data.x;
    double y  = obs_data.y;

    for(auto &p : vertices) {
      double xa = p.x*ct - p.y*st + x;
      double ya = p.x*st + p.y*ct + y;
      p.x = xa;
      p.y = ya;
    }

    if (obs_data.type == Obstacle::ObstacleType::kSolid) {
      return std::make_shared<Obstacle>(vertices);
    } else if (obs_data.type == Obstacle::ObstacleType::kDynamic) {
      return std::make_shared<DynamicObstacle>(vertices, 1);
    } else if (obs_data.type == Obstacle::ObstacleType::kVibration) { 
      return std::make_shared<VibrationObstacle>(vertices, obs_data.extra_info.at("roughness"));
    } else {
      return std::make_shared<Obstacle>(Obstacle::ObstacleType::kDoor, vertices);
    }
  } else {
    if (obs_data.type == Obstacle::ObstacleType::kSolid) {
      return std::make_shared<Obstacle>(Point<double>(obs_data.x, obs_data.y), obs_data.radius);
    } else if (obs_data.type == Obstacle::ObstacleType::kDynamic) {
      return std::make_shared<DynamicObstacle>(Point<double>(obs_data.x, obs_data.y), obs_data.radius, 1);
    } else if (obs_data.type == Obstacle::ObstacleType::kVibration) { 
      return std::make_shared<VibrationObstacle>(Point<double>(obs_data.x, obs_data.y), obs_data.radius, obs_data.extra_info.at("roughness"));
    } else {
      return std::make_shared<Obstacle>(Obstacle::ObstacleType::kDoor, Point<double>(obs_data.x, obs_data.y), obs_data.radius);
    }
  }
}

Obstacle solid_obs_from_setup_data(const ObstacleSetupData &obs_data)
{
  std::shared_ptr<Obstacle> obs = obs_from_setup_data(obs_data);
  return *obs;
}

std::vector<std::shared_ptr<Obstacle>> obstacles_from_file(const std::string &json_file)
{
  std::vector<ObstacleSetupData> obs_data = obstacle_data_from_file(json_file);
  
  std::vector<std::shared_ptr<Obstacle>> obs;
  for(const auto &o : obs_data) {
    obs.push_back(obs_from_setup_data(o));
  }
  return obs;
}

std::vector<Obstacle> solid_obstacles_from_file(const std::string &json_file)
{
  std::vector<ObstacleSetupData> obs_data = obstacle_data_from_file(json_file);
  
  std::vector<Obstacle> obs;
  for(const auto &o : obs_data) {
    if(o.type == Obstacle::ObstacleType::kSolid) {
      obs.push_back(solid_obs_from_setup_data(o));   
    }
  }
  return obs;
}

std::map<std::string, std::vector<double>> obstacle_vicon_data(ros::NodeHandle &nh, const std::string &filename)
{
  std::vector<ObstacleSetupData> file_obs = obstacle_data_from_file(filename);
  std::map<std::string, std::vector<double>> obs_movement;

  // Wait a few seconds in case Vicon bridge is currently starting up
  if(!util::topic_is_advertised("/vicon/markers")) {
    ros::Rate(1).sleep();
    ros::Rate(1).sleep();
    ros::Rate(1).sleep();
  }

  for (auto &elem : file_obs) {
    if ((!elem.label.empty()) && elem.type == Obstacle::ObstacleType::kSolid) {

      std::string topic = "/vicon/" + elem.label + "/" + elem.label;
      ROS_INFO("Looking for obstacle topic: %s", topic.c_str());

      boost::function<void(const geometry_msgs::TransformStamped::ConstPtr&)> obs_cb = 
        boost::bind(obstacleCallback, _1, boost::ref(elem));
      ros::Subscriber sub = nh.subscribe(topic, 1, obs_cb);

      int count = 0;
      while (ros::ok() && !elem.pos_set && (count++) < 15) {
        ros::spinOnce();
        ros::Rate(10).sleep();
      }

      if (elem.pos_set)  {  
        obs_movement[elem.label] = std::vector<double>(3);
        obs_movement[elem.label][0] = elem.x;
        obs_movement[elem.label][1] = elem.y;
        obs_movement[elem.label][2] = elem.theta;
      }

      // Not strictly neccessary, but felt like being 
      // explicit that subcriber is no longer needed.
      sub.shutdown();
    }
  }

  return obs_movement;
}

std::vector<Obstacle> obstacles_with_vicon(ros::NodeHandle &nh, const std::string &filename)
{
  std::vector<ObstacleSetupData> file_obs = obstacle_data_from_file(filename);
  
  // Obstacles that are in file but not being published by vicon.
  // Saved to erase once loop is done.
  std::vector<std::string> ignore_obs; 

  // Wait a few seconds if Vicon bridge isn't up yet
  if(!util::topic_is_advertised("/vicon/markers")) {
    ros::Rate(1).sleep();
    ros::Rate(1).sleep();
    ros::Rate(1).sleep();
  }

  for (auto &elem : file_obs) {
    if ((!elem.label.empty()) && elem.type == Obstacle::ObstacleType::kSolid) {

      std::string topic = "/vicon/" + elem.label + "/" + elem.label;
      ROS_INFO("Looking for obstacle topic: %s", topic.c_str());
      
      boost::function<void(const geometry_msgs::TransformStamped::ConstPtr&)> obs_cb = 
        boost::bind(obstacleCallback, _1, boost::ref(elem));
      ros::Subscriber sub = nh.subscribe(topic, 1, obs_cb);

      int count = 0;
      while (ros::ok() && !elem.pos_set && (count++) < 15) {
        ros::spinOnce();
        ros::Rate(10).sleep();
      }

      if (!elem.pos_set)  {  ignore_obs.push_back(elem.label); }

      // Not strictly neccessary, but felt like being 
      // explicit that subcriber is no longer needed.
      sub.shutdown();
    } else {
      ignore_obs.push_back(elem.label);
    }
  }

  for(const auto &ignore : ignore_obs) { 
    ROS_INFO("Erase: %s", ignore.c_str());
    auto new_end = std::remove_if(file_obs.begin(), file_obs.end(), [ignore](const auto &o) { 
      return o.label.empty() || (o.label == ignore); });
    file_obs.erase(new_end, file_obs.end()); // remove_if doesn't actually erase elements. Just rearranges.
  }

  std::vector<Obstacle> obs;
  for(const auto &o : file_obs) { obs.push_back(solid_obs_from_setup_data(o)); }

  return obs;
}

std::vector<ObstacleSetupData> obstacle_data_from_file(const std::string &json_file)
{
  using json = nlohmann::json;

  std::ifstream f(json_file, std::ifstream::in);
  if(f.is_open()) {
    json data = json::parse(f);
    if(!data.empty()) { 
      std::vector<ObstacleSetupData> obs_data;

      if(data.contains("obstacles")) {
        json obs_json = data["obstacles"];
        std::vector<ObstacleSetupData> solid_obs = obs_data_from_json_obj(obs_json, Obstacle::ObstacleType::kSolid);
        obs_data.insert(obs_data.end(), solid_obs.begin(), solid_obs.end());
      } 

      if(data.contains("dynamic")) {
        json obs_json = data["dynamic"];
        std::vector<ObstacleSetupData> dyn_obs = obs_data_from_json_obj(obs_json, Obstacle::ObstacleType::kDynamic);
        obs_data.insert(obs_data.end(), dyn_obs.begin(), dyn_obs.end());
      }

      if(data.contains("vibration")) {
        json obs_json = data["vibration"];
        std::vector<ObstacleSetupData> misc_obs = obs_data_from_json_obj(obs_json, Obstacle::ObstacleType::kVibration);
        obs_data.insert(obs_data.end(), misc_obs.begin(), misc_obs.end());
      }

      if(data.contains("misc")) {
        json obs_json = data["misc"];
        std::vector<ObstacleSetupData> misc_obs = obs_data_from_json_obj(obs_json, Obstacle::ObstacleType::kDoor);
        obs_data.insert(obs_data.end(), misc_obs.begin(), misc_obs.end());
      }

      return obs_data;
    } else {
      ROS_WARN("Invalid JSON file: %s", json_file.c_str()); 
    }
  } else {
    ROS_WARN("JSON file NOT open (likely invalid filename): %s", json_file.c_str()); 
  }

  return {};
}

std::vector<ObstacleSetupData> obs_data_from_json_obj(nlohmann::json &obs_json, const Obstacle::ObstacleType obs_type)
{
  using json = nlohmann::json;

  std::vector<ObstacleSetupData> obstacles;

  for(json::iterator it = obs_json.begin(); it != obs_json.end(); ++it) {
    ObstacleSetupData obs;

    if(it->contains("shape") && (it->at("shape") == "circle")) {
      obs.shape = ObstacleShape::ShapeType::kCircle;
    } else {
      obs.shape = ObstacleShape::ShapeType::kPolygon;
    }

    if(obs.shape == ObstacleShape::ShapeType::kPolygon) {
      json vertices = it->at("vertices");
      for(json::iterator iit = vertices.begin(); iit != vertices.end(); ++iit) {
        amrl::Point<double> p(iit.value()[0], iit.value()[1]);
        obs.vertices.push_back(p);
      }
    } else {
      obs.radius = it->at("radius");
    }
 
    if(it->contains("offset")) {
      obs.x = it->at("offset")[0];
      obs.y = it->at("offset")[1];
    } else {
      obs.x = 0.0;
      obs.y = 0.0;
    }

    if(it->contains("heading")) {
      obs.theta = it->at("heading");
    } else {
      obs.theta = 0.0;
    }

    if(it->contains("label")) {
      obs.label = it->at("label");
    } else {
      obs.label = "obs" + std::to_string(obstacles.size());
    }

    if(it->contains("detectable")) {
      obs.detectable = it->at("detectable");
    } else {
      obs.detectable = true;
    }

    if(it->contains("color")) {
      obs.color = it->at("color");
    } else {
      obs.color = "";
    }

    if(it->contains("alpha")) {
      obs.alpha = it->at("alpha");
    } else {
      obs.alpha = 1.0;
    }

    obs.type = obs_type;
    if(obs.type == Obstacle::ObstacleType::kVibration) {
      obs.extra_info["roughness"] = it->at("roughness");
    } 

    obstacles.push_back(obs);
  }

  return obstacles;
}

std::vector<Point<double>> features_from_obs_file(const std::string &json_file)
{
  std::vector<amrl::Obstacle> obs = solid_obstacles_from_file(json_file);
  size_t N = obs.size();

  std::vector<Point<double>> features;
  for(size_t i = 0; i < N; ++i) {
    const amrl::Obstacle &o  = obs[i];
    if(o.shape_get() == ObstacleShape::ShapeType::kCircle) {
      std::vector<double> data = o.data_get();
      Point<double> pt({data[0], data[1]});
      features.emplace_back(Point<double>(data[0], data[1]));
    }
  }

  return features;
}

bool topic_is_advertised(const std::string &topic)
{
  ros::master::V_TopicInfo master_topics;
  if(ros::master::getTopics(master_topics)) {
    for(const auto &tpc : master_topics) {
      if(tpc.name == topic) { return true; } 
    }
  }
  return false;
}


} // namespace util
} // namespace amrl