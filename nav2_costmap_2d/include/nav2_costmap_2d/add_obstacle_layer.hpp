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
#ifndef ADD_OBSTACLE_LAYER_HPP_
#define ADD_OBSTACLE_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include <mutex>
#include "geometry_msgs/msg/point.hpp"

namespace nav2_costmap_2d
{

class AddObstacleLayer : public nav2_costmap_2d::Layer
{
public:
  AddObstacleLayer();
  
  /**
    * @brief A destructor
    */
  ~AddObstacleLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void reset()
  {
    return;
  }

  virtual void onFootprintChanged();

  virtual bool isClearable() {return false;}

  // Provide a typedef to ease future code maintenance
  typedef std::recursive_mutex mutex_t;

  /**
   * @brief Get the mutex of the inflation inforamtion
   */
  mutex_t * getMutex()
  {
    return access_;
  }

  void obstacles_callback(const geometry_msgs::msg::Point::SharedPtr msg);

private:
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_, check_wx_, check_wy_, msg_wx_, msg_wy_;

  mutex_t * access_;

  // Indicates that the entire gradient should be recalculated next time.
  bool need_recalculation_;

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr obstacle_sub_;

  std::vector<std::pair<double, double>> obstacle_vec_;  // x & y coordinates of obstacles in map frame

  double update_window_height_m_;
  double update_window_width_m_;
};

}  // namespace nav2_costmap_2d

#endif  // ADD_OBSTACLE_LAYER_HPP_
