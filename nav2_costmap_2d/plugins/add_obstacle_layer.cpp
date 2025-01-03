/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2020, Samsung R&D Institute Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Alexey Merzlyakov
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
 *********************************************************************/
#include "nav2_costmap_2d/add_obstacle_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_costmap_2d
{

AddObstacleLayer::AddObstacleLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max()),
  check_wx_(-std::numeric_limits<float>::max()),
  check_wy_(-std::numeric_limits<float>::max()),
  msg_wx_(-std::numeric_limits<float>::max()),
  msg_wy_(-std::numeric_limits<float>::max()),
  obstacle_sub_(nullptr),
  update_window_height_m_(1.0),
  update_window_width_m_(1.0)
{
  access_ = new mutex_t();
}

AddObstacleLayer::~AddObstacleLayer()
{
  delete access_;
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void
AddObstacleLayer::onInitialize()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  
  auto node = node_.lock(); 
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);
  
  declareParameter("update_window_height_m", rclcpp::ParameterValue(update_window_height_m_));
  node->get_parameter(name_ + "." + "update_window_height_m", update_window_height_m_);

  RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "AddObstacleLayer: update_window_height_m: %f", update_window_height_m_);

  declareParameter("update_window_width_m", rclcpp::ParameterValue(update_window_width_m_));
  node->get_parameter(name_ + "." + "update_window_width_m", update_window_width_m_);

  RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "AddObstacleLayer: update_window_width_m: %f", update_window_width_m_);

  obstacle_sub_ = node->create_subscription<geometry_msgs::msg::Point>(
    "/summit/sensor_obstacles", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&AddObstacleLayer::obstacles_callback, this, std::placeholders::_1));

  need_recalculation_ = false;
  current_ = true;
}

void AddObstacleLayer::obstacles_callback(const geometry_msgs::msg::Point::SharedPtr msg)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  msg_wx_ = msg->x;
  msg_wy_ = msg->y;

  RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "AddObstacleLayer: received obstacle in map frame: x: %f, y: %f", msg_wx_, msg_wy_);
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void
AddObstacleLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  if (need_recalculation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void
AddObstacleLayer::onFootprintChanged()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  
  need_recalculation_ = true;

  RCLCPP_INFO(rclcpp::get_logger(
      "nav2_costmap_2d"), "AddObstacleLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
void
AddObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  
  if (!enabled_) {
    return;
  }

  std::string obstacle_frame = "map";  
  std::string costmap_frame = layered_costmap_->getGlobalFrameID();

  double Resolution_m_p_c = master_grid.getResolution();
  // RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "AddObstacleLayer: Resolution (m/cell): %f", Resolution_m_p_c);
  
  if ((msg_wx_ == -std::numeric_limits<float>::max()) || (msg_wy_ == -std::numeric_limits<float>::max()))
  {
    // RCLCPP_WARN(rclcpp::get_logger("nav2_costmap_2d"), "AddObstacleLayer: no obstacle coordinates received yet");
    return;
  }
  
  else if(std::fabs(msg_wx_ - check_wx_) < Resolution_m_p_c && std::fabs(msg_wy_ - check_wy_) < Resolution_m_p_c)
  {
    // RCLCPP_ERROR(rclcpp::get_logger("nav2_costmap_2d"), "AddObstacleLayer: received similar obstacle coordinates, not adding to the obstacle list!");
  }   

  else
  {
    check_wx_ = msg_wx_;
    check_wy_ = msg_wy_;

    std::pair<double, double> check_wxy(check_wx_, check_wy_);
    obstacle_vec_.push_back(check_wxy); 
  }

  int map_x = 0;
  int map_y = 0;
  unsigned int i, j = 0;  // master_grid iterators

  int update_window_height_cell = static_cast<int>(std::round(update_window_height_m_ / (Resolution_m_p_c * 2.0))); 
  int update_window_width_cell = static_cast<int>(std::round(update_window_width_m_ / (Resolution_m_p_c * 2.0))); 

  int map_x_min = 0;
  int map_y_min = 0;
  int map_x_max = 0;
  int map_y_max = 0;

  unsigned int map_x_min_u = 0;
  unsigned int map_y_min_u = 0;
  unsigned int map_x_max_u = 0;
  unsigned int map_y_max_u = 0;

  if (!obstacle_vec_.empty()) 
  {
    for (size_t k = 0; k < obstacle_vec_.size(); ++k) {
      std::pair<double, double> point = obstacle_vec_[k];
      double wx = point.first;  
      double wy = point.second;
  
      // RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "AddObstacleLayer: costmap_frame: %s", costmap_frame.c_str()); 

      tf2::Transform tf2_transform;
      tf2_transform.setIdentity();  // initialize by identical transform

      if (obstacle_frame != costmap_frame) {

        // RCLCPP_WARN(
        //     rclcpp::get_logger("nav2_costmap_2d"),
        //     "AddObstacleLayer: obstacle (%s) and costmap frame (%s) are different, transforming coordinates to costmap frame",
        //     obstacle_frame.c_str(), costmap_frame.c_str());

        geometry_msgs::msg::TransformStamped transform;
        try {
          transform = tf_->lookupTransform(
            obstacle_frame, costmap_frame, tf2::TimePointZero,
            tf2::durationFromSec(5.0));
          
          tf2::fromMsg(transform.transform, tf2_transform);

          tf2::Vector3 point(wx, wy, 0);
          point = tf2_transform * point;
          wx = point.x();
          wy = point.y();
        } catch (tf2::TransformException & ex) {
          RCLCPP_ERROR(
            rclcpp::get_logger("nav2_costmap_2d"),
            "AddObstacleLayer: Failed to get transformation from %s to %s: %s",
            obstacle_frame.c_str(), costmap_frame.c_str(), ex.what());
          return;
        }
      }

      // RCLCPP_INFO(
      //   rclcpp::get_logger("nav2_costmap_2d"),
      //   "AddObstacleLayer: Coordinates in costmap_frame (%s): x: %f, y: %f",
      //   costmap_frame.c_str(), wx, wy);

      master_grid.worldToMapNoBounds(wx, wy, map_x, map_y);
      
      if (map_x <= min_i || map_y <= min_j || map_x >= max_i || map_y >= max_j) {
        // RCLCPP_WARN(rclcpp::get_logger("nav2_costmap_2d"), "AddObstacleLayer: map coordinates out of bounds, do nothing");
        return;
      }

      map_x_min = std::max(min_i, map_x - update_window_height_cell);
      map_y_min = std::max(min_j, map_y - update_window_width_cell);
      map_x_max = std::min(max_i, map_x + update_window_height_cell);
      map_y_max = std::min(max_j, map_y + update_window_width_cell);

      // unsigned<-signed conversions.
      map_x_min_u = static_cast<unsigned int>(map_x_min);
      map_y_min_u = static_cast<unsigned int>(map_y_min);
      map_x_max_u = static_cast<unsigned int>(map_x_max);
      map_y_max_u = static_cast<unsigned int>(map_y_max);
      
      // Main master_grid updating loop

      for (i = map_x_min_u; i < map_x_max_u; i++) {
        for (j = map_y_min_u; j < map_y_max_u; j++) {
          unsigned char cost = master_grid.getCost(i, j);

          if (cost != LETHAL_OBSTACLE)
          {
            master_grid.setCost(i, j, LETHAL_OBSTACLE);
          }
        }
      }
    }
  }
}

}  // namespace nav2_costmap_2d

// This is the macro allowing a nav2_costmap_2d::AddObstacleLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::AddObstacleLayer, nav2_costmap_2d::Layer)
