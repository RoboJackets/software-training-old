// Copyright 2021 RoboJackets
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <algorithm>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
// BEGIN STUDENT CODE
// END STUDENT CODE

namespace mapping
{

double toLogOdds(double prob)
{
  return std::log(prob / (1.0 - prob));
}

double fromLogOdds(double log_odds)
{
  return 1.0 - (1.0 / (1.0 + std::exp(log_odds)));
}

class MappingNode : public rclcpp::Node
{
public:
  explicit MappingNode(const rclcpp::NodeOptions & options)
  // BEGIN STUDENT CODE
  : rclcpp::Node("mapping_node", options)
    // END STUDENT CODE
  {
    map_frame_id_ = declare_parameter<std::string>("map_frame", "map");
    robot_frame_id_ = declare_parameter<std::string>("robot_frame", "base_link");
    distance_coefficient_ = declare_parameter<double>("distance_coefficient", 0.01);
    hit_log_odds_ = toLogOdds(declare_parameter<double>("hit_probability", 0.7));
    miss_log_odds_ = toLogOdds(declare_parameter<double>("miss_probability", 0.3));

    const auto map_width = declare_parameter<double>("map_width", 1.2192);  // meters
    const auto map_height = declare_parameter<double>("map_height", 0.762);  // meters
    const auto map_resolution = declare_parameter<double>("map_resolution", 0.01);  // meters/cell

    map_info_.origin.position.x = -map_width / 2.0;
    map_info_.origin.position.y = -map_height / 2.0;
    map_info_.height = map_height / map_resolution;
    map_info_.width = map_width / map_resolution;
    map_info_.resolution = map_resolution;

    const auto map_data_size = map_info_.height * map_info_.width;
    map_data_.reserve(map_data_size);
    std::fill_n(std::back_inserter(map_data_), map_data_size, 0.0);

    map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "~/map",
      rclcpp::SystemDefaultsQoS());

    // BEGIN STUDENT CODE
    // END STUDENT CODE
  }

private:
  // BEGIN STUDENT CODE
  // END STUDENT CODE
  std::string map_frame_id_;
  std::string robot_frame_id_;
  double distance_coefficient_;
  double hit_log_odds_;
  double miss_log_odds_;
  std::vector<double> map_data_;
  nav_msgs::msg::MapMetaData map_info_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacles_subscription_;

  void ObstaclesCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr obstacles_msg)
  {
    // BEGIN STUDENT CODE
    // END STUDENT CODE

    AddObstaclesToMap(*obstacles_msg);

    nav_msgs::msg::OccupancyGrid map_msg;
    map_msg.header.frame_id = map_frame_id_;
    const auto current_time = now();
    map_msg.header.stamp = current_time;
    map_msg.info = map_info_;
    map_msg.info.map_load_time = current_time;

    std::transform(
      map_data_.begin(), map_data_.end(), std::back_inserter(map_msg.data), [](
        const auto & percent) {
        return static_cast<int8_t>(fromLogOdds(percent) * 100);
      });

    map_publisher_->publish(map_msg);
  }

  geometry_msgs::msg::Point GetRobotLocation()
  {
    // BEGIN STUDENT CODE
    return geometry_msgs::msg::Point{};
    // END STUDENT CODE
  }

  void AddObstaclesToMap(const nav_msgs::msg::OccupancyGrid & obstacles)
  {
    const auto robot_location = GetRobotLocation();

    // BEGIN STUDENT CODE
    // END STUDENT CODE
  }

  void UpdateProbability(
    const geometry_msgs::msg::Point & robot_location,
    const geometry_msgs::msg::Point & map_location,
    const bool & obstacle_detected)
  {
    // BEGIN STUDENT CODE
    // END STUDENT CODE
  }

  bool IsLocationInMapBounds(const geometry_msgs::msg::Point & location)
  {
    return location.x >= map_info_.origin.position.x &&
           location.x < (map_info_.origin.position.x + (map_info_.width * map_info_.resolution)) &&
           location.y >= map_info_.origin.position.y &&
           location.y < (map_info_.origin.position.y + (map_info_.height * map_info_.resolution));
  }

  std::size_t MapDataIndexFromLocation(const int cell_x, const int cell_y)
  {
    assert(cell_x >= 0 && cell_x < map_info_.width);
    assert(cell_y >= 0 && cell_y < map_info_.height);
    return cell_x + (cell_y * map_info_.width);
  }
};

}  // namespace mapping

RCLCPP_COMPONENTS_REGISTER_NODE(mapping::MappingNode)
