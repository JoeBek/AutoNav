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
#include "line_layer/line_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

template class LineBuffer<std::shared_ptr<autonav_interfaces::msg::LinePoints>>;

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

//#define DEBUG_
//#define DEBUG_2
//#define DEBUG_3
//#define DEBUG_4

// helper methods outside namespace

template <typename T>
bool within_bounds(T value, T min, T max) {

  return (value >= min) && (value <= max);
}


namespace line_layer
{

LineLayer::LineLayer()
: last_min_x_(0.0),
  last_min_y_(0.0),
  last_max_x_(1.0),
  last_max_y_(1.0)
{
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void
LineLayer::onInitialize()
{
  auto node = node_.lock(); 
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("line_topic", rclcpp::ParameterValue("line_points"));
  declareParameter("rolling_window", rclcpp::ParameterValue(false));
  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "line_topic", line_topic_);
  node->get_parameter(name_ + "." + "rolling_window", rolling_window_);
  node->get_parameter(name_ + "." + "publish_costmap", publish_costmap_);


  line_sub_ = node->create_subscription<autonav_interfaces::msg::LinePoints>(line_topic_, 1, 
    std::bind(&LineLayer::linePointCallback, this, std::placeholders::_1));
  
  if (publish_costmap_) {
    costmap_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/line_costmap", 1);
  }

  

  need_recalculation_ = false;
  current_ = true;
  
  RCLCPP_INFO(rclcpp::get_logger("nav_costmap_2d"), "hello from line land");
  
}
/// @brief I just accidentally did this wtf.... this is a line callback that mimics the cool other costmap plugins
/// @param message 
/// @param buffer 
void LineLayer::linePointCallback(autonav_interfaces::msg::LinePoints::ConstSharedPtr message) {

      #ifdef DEBUG_
      RCLCPP_INFO(rclcpp::get_logger("nav_costmap_2d"), "CALM LUH CALLBACK");
      #endif
      auto line = std::make_shared<autonav_interfaces::msg::LinePoints>(); 
      line->points = message->points;

      buffer_.buffer(line);

}

void LineLayer::publishCostmap() {

  auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
  
  msg->header.frame_id = layered_costmap_->getGlobalFrameID();
  msg->header.stamp = node_.lock()->now();
  msg->info.width = size_x_;
  msg->info.height = size_y_;
  msg->info.origin.position.x = origin_x_;
  msg->info.origin.position.y = origin_y_;
  msg->data.resize(size_x_ * size_y_);
  for (unsigned int i = 0; i < size_x_ * size_y_; ++i) {
    unsigned char cost = costmap_[i];
    if (cost == NO_INFORMATION) {
      msg->data[i] = -1;

    }
    else {
      msg->data[i] = static_cast<int8_t>(cost * 100 / 254);
      
    }

  }
  costmap_pub_->publish(std::move(msg));
  
}

// used in obstacle layer and voxel layer to correct bounds for the local costmap. 
void LineLayer::updateOrigin(double new_origin_x, double new_origin_y)
{
  if (!costmap_)
	  return;
  // project the new origin into the grid
  int cell_ox, cell_oy;
  cell_ox = static_cast<int>((new_origin_x - origin_x_) / resolution_);
  cell_oy = static_cast<int>((new_origin_y - origin_y_) / resolution_);

  // compute the associated world coordinates for the origin cell
  // because we want to keep things grid-aligned
  double new_grid_ox, new_grid_oy;
  new_grid_ox = origin_x_ + cell_ox * resolution_;
  new_grid_oy = origin_y_ + cell_oy * resolution_;

  // To save casting from unsigned int to int a bunch of times
  int size_x = size_x_;
  int size_y = size_y_;

  // we need to compute the overlap of the new and existing windows
  int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  lower_left_x = std::min(std::max(cell_ox, 0), size_x);
  lower_left_y = std::min(std::max(cell_oy, 0), size_y);
  upper_right_x = std::min(std::max(cell_ox + size_x, 0), size_x);
  upper_right_y = std::min(std::max(cell_oy + size_y, 0), size_y);

  unsigned int cell_size_x = upper_right_x - lower_left_x;
  unsigned int cell_size_y = upper_right_y - lower_left_y;

  // we need a map to store the obstacles in the window temporarily
  unsigned char * local_map = new unsigned char[cell_size_x * cell_size_y];

  // copy the local window in the costmap to the local map
  copyMapRegion(
    costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x,
    cell_size_x,
    cell_size_y);

  // we'll reset our maps to unknown space if appropriate
  resetMaps();

  // update the origin with the appropriate world coordinates
  origin_x_ = new_grid_ox;
  origin_y_ = new_grid_oy;

  // compute the starting cell location for copying data back in
  int start_x = lower_left_x - cell_ox;
  int start_y = lower_left_y - cell_oy;

  // now we want to copy the overlapping information back into the map, but in its new location
  copyMapRegion(
    local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x,
    cell_size_y);
  // make sure to clean up
  delete[] local_map;
}




// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void
LineLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  if (need_recalculation_) {
    
    if (rolling_window_) {

      updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
    }

    

    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    //*min_x = -std::numeric_limits<float>::max();
    //*min_y = -std::numeric_limits<float>::max();
    //*max_x = std::numeric_limits<float>::max();
    //*max_y = std::numeric_limits<float>::max();

      // Set a 20x20 meter area around the robot
    double half_size = 10.0; // 10 meters in each direction = 20x20 total
    
    *min_x = std::min(*min_x, robot_x - half_size);
    *min_y = std::min(*min_y, robot_y - half_size);
    *max_x = std::max(*max_x, robot_x + half_size);
    *max_y = std::max(*max_y, robot_y + half_size);

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
LineLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "LineLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void
LineLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }
  if (!costmap_) 
	  return;

  // master_array - is a direct pointer to the resulting master_grid.
  // master_grid - is a resulting costmap combined from all layers.
  // By using this pointer all layers will be overwritten!
  // To work with costmap layer and merge it with other costmap layers,
  // please use costmap_ pointer instead (this is pointer to current
  // costmap layer grid) and then call one of updates methods:
  // - updateWithAddition()
  // - updateWithMax()
  // - updateWithOverwrite()
  // - updateWithTrueOverwrite()
  // In this case using master_array pointer is equal to modifying local costmap_
  // pointer and then calling updateWithTrueOverwrite():

  // below is a testament to my stupidity. Do not be like me. There is always a reason they have it set up the way they do.

  // Idgaf I'm overwriting just like they did
  //unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
  // These variables are used to update the costmap only within this window
  // avoiding the updates of whole area.
  //
  // Fixing window coordinates with map size if necessary.
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  // joe was here

  // std::vector<geometry_msgs::msg::Vector3> points;
  #ifdef DEBUG_
  RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "HEEEEEEEEEEEELP HEEEELP ME HEEEEEEEEEELP");
  #endif

  // why even use the name thingys if auto works for all of them
  auto last = buffer_.read();
  if (!last ){
    RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "buffa empty... nothing to buf");
    return;
  }
  auto last_msg = *last;
  if (!last_msg) {
    RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "last message is gone... in the wind");
    return;
  }
  
  std::vector<geometry_msgs::msg::Vector3> points = last_msg->points;

  #ifdef DEBUG_2
  RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "line point len: %zu", points.size());
  #endif


  
  // add points to costmap, include bounds checking
  for (auto &point : points) {
    // now we need to compute the map coordinates for the observation


    double x = point.x;
    double y = point.y;

    #ifdef DEBUG_3 
    RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "x, y = (%f, %f)", x, y);
    #endif

    unsigned int mx, my;
    if (!master_grid.worldToMap(x, y, mx, my)) {
      
      #ifdef DEBUG_
        RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "grid coords: (%u,%u)", mx, my); 
        //RCLCPP_WARN(rclcpp::get_logger("nav_costmap_2d"), "LISTEN UP Computing map coords failed");
      #endif
    }


    if (!within_bounds(static_cast<int>(mx), min_i, max_i) || !within_bounds(static_cast<int>(my), min_j, max_j)) {

      #ifdef DEBUG_4
      RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "bounds: (%d, %d), (%d, %d)",min_i, max_i, min_j, max_j); 
      RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "input: (%u), (%u)", mx, my); 
      #endif
      continue;
    }
    //int index_costmap = master_grid.getIndex(mx, my);
    int index_new = my * size_x_ + mx;
    //costmap_->setCost(mx, my, LETHAL_OBSTACLE);
    unsigned char cost = LETHAL_OBSTACLE; // maybe more dynamic down the line
    //master_array[index_costmap] = cost; // overwrites cost map
    costmap_[index_new] = cost; // overwrites cost map

    #ifdef DEBUG_
    RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "grid coords: (%u,%u)", mx, my); 
    #endif



  }

  updateWithMax(master_grid, min_i, min_j, max_i, max_j);

  if (publish_costmap_) {
   publishCostmap();
  }
  
}




}  // namespace nav2_gradient_costmap_plugin

// This is the macro allowing a nav2_gradient_costmap_plugin::LineLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(line_layer::LineLayer, nav2_costmap_2d::Layer)
