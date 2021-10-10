#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>

namespace mapping
{

class MappingNode : public rclcpp::Node
{
public:
  explicit MappingNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("mapping_node", options)
  {
    map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "~/map",
      rclcpp::SystemDefaultsQoS());
    timer_ =
      create_wall_timer(std::chrono::seconds(1), std::bind(&MappingNode::TimerCallback, this));
  }

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void TimerCallback()
  {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.frame_id = "map";
    const auto current_time = now();
    msg.header.stamp = current_time;
    msg.info.map_load_time = current_time;

    constexpr auto map_width = 1.2192;  // m
    constexpr auto map_height = 0.762;  // m
    constexpr auto map_resolution = 0.01;  // m/cell

    msg.info.origin.position.x = -map_width / 2.0;
    msg.info.origin.position.y = -map_height / 2.0;
    msg.info.height = map_height / map_resolution;
    msg.info.width = map_width / map_resolution;
    msg.info.resolution = map_resolution;

    const auto map_data_size = static_cast<int>(map_width / map_resolution) * static_cast<int>(map_height / map_resolution);

    msg.data = std::vector<int8_t>(map_data_size, 0);

    map_publisher_->publish(msg);
  }

};

}  // namespace mapping

RCLCPP_COMPONENTS_REGISTER_NODE(mapping::MappingNode)
