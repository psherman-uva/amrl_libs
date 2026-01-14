/*
  @file:      FigureDisplay.hpp
  @author:    psherman
  @date       Jan 2026
  
  @brief Class for managing a display figure
*/

#pragma once

#include <amrl_display/CreateFigure.h>
#include <amrl_display/DeleteElement.h>

#include <ros/ros.h>

namespace amrl {

class FigureDisplayManager
{
public:

  /// Constructor
  /// @param nh ROS nodehandle object
  FigureDisplayManager(ros::NodeHandle &nh);

    /// Destructor
  ~FigureDisplayManager(void) = default;

  void add_from_ros_params(const std::string &ns);


  // ----------------------------------- //
  // --    Public Class Methods       -- //
  // ----------------------------------- //

private:
  // ----------------------------------- //
  // --    Private Class Methods      -- //
  // ----------------------------------- //


  // ----------------------------------- //
  // --     Class Member Variables    -- //
  // ----------------------------------- //

  /// ROS Node Handle object
  ros::NodeHandle _nh;


  /// Client to create a new display figure
  ros::ServiceClient _fig_client;


};


}