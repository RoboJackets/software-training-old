#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace controller_test_client
{

class ControllerTestClient : public rclcpp::Node
{
public:

explicit ControllerTestClient(const rclcpp::NodeOptions & options)
: rclcpp::Node("controller_test_client", options)
{
}

}; 
} // namespace controller_test_client
RCLCPP_COMPONENTS_REGISTER_NODE(controller_test_client::ControllerTestClient)