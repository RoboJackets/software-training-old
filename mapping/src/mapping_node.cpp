#include <algorithm>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace mapping
{

double toLogOdds(double prob) {
  return std::log(prob / (1.0 - prob));
}

double fromLogOdds(double log_odds) {
  return 1.0 - (1.0 / (1.0 + std::exp(log_odds)));
}

class MappingNode : public rclcpp::Node
{
public:
  explicit MappingNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("mapping_node", options),
    // BEGIN STUDENT CODE
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
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
    obstacles_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "~/obstacles",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&MappingNode::ObstaclesCallback, this, std::placeholders::_1));
  }

private:
  // BEGIN STUDENT CODE
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
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
    if(!tf_buffer_.canTransform(robot_frame_id_, map_frame_id_, tf2::TimePointZero) || !tf_buffer_.canTransform(obstacles_msg->header.frame_id, map_frame_id_, tf2::TimePointZero)) {
      RCLCPP_INFO_ONCE(get_logger(), "Waiting for necessary TF data to be available.");
      return;
    }
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
    const auto robot_transform = tf_buffer_.lookupTransform(robot_frame_id_, map_frame_id_, tf2::TimePointZero);
    geometry_msgs::msg::Point robot_location;
    robot_location.x = robot_transform.transform.translation.x;
    robot_location.y = robot_transform.transform.translation.y;
    return robot_location;
    // END STUDENT CODE
  }

  void AddObstaclesToMap(const nav_msgs::msg::OccupancyGrid& obstacles)
  {
    const auto robot_location = GetRobotLocation();

    // BEGIN STUDENT CODE
    for (auto y = 0; y < obstacles.info.height; ++y) {
      for (auto x = 0; x < obstacles.info.width; ++x) {
        const auto obstacle_data_index = x + (y * obstacles.info.width);
        const auto obstacle_data = obstacles.data.at(obstacle_data_index);

        if (obstacle_data == -1) {
          continue;
        }

        geometry_msgs::msg::PointStamped obstacle_location;
        obstacle_location.header.frame_id = obstacles.header.frame_id;
        obstacle_location.point.x = (x * obstacles.info.resolution) +
          obstacles.info.origin.position.x;
        obstacle_location.point.y = (y * obstacles.info.resolution) +
          obstacles.info.origin.position.y;

        const auto map_location = tf_buffer_.transform(obstacle_location, map_frame_id_);

        if(!IsLocationInMapBounds(map_location.point)) {
          // skip any detections that are outside of map bounds
          continue;
        }

        UpdateProbability(robot_location, map_location.point, obstacle_data == 100);
      }
    }
    // END STUDENT CODE
  }

  void UpdateProbability(
    const geometry_msgs::msg::Point & robot_location,
    const geometry_msgs::msg::Point & map_location,
    const bool & obstacle_detected)
  {
    // BEGIN STUDENT CODE
    const int cell_x = (map_location.x - map_info_.origin.position.x) / map_info_.resolution;
    const int cell_y = (map_location.y - map_info_.origin.position.y) / map_info_.resolution;
    const auto data_index = MapDataIndexFromLocation(cell_x, cell_y);

    const auto distance_robot_to_measurement = std::hypot(
      robot_location.x - map_location.x,
      robot_location.y - map_location.y);

    auto probability = std::exp(-distance_coefficient_ * distance_robot_to_measurement);

    probability *= (obstacle_detected ? hit_log_odds_ : miss_log_odds_);

    map_data_[data_index] += probability;

    map_data_[data_index] = std::clamp(map_data_[data_index], 0.0, 1.0);
    // END STUDENT CODE
  }

  bool IsLocationInMapBounds(const geometry_msgs::msg::Point& location) {
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
