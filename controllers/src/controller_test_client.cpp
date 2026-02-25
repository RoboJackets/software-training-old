#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "test_path_generator.hpp"

namespace controllers
{

class ControllerTestClient : public rclcpp::Node
{
using FollowPath = nav2_msgs::action::FollowPath;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

public:

    explicit ControllerTestClient(const rclcpp::NodeOptions & options)
    : rclcpp::Node("controller_test_client", options)
    {
        client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");

        rclcpp::QoS qos_profile(1);
        qos_profile.transient_local();

        publisher_ = create_publisher<nav_msgs::msg::Path>(
            "/plan",
            qos_profile);

        std::thread(&ControllerTestClient::SendGoal, this).detach();
    }

private:
    void SendGoal()
    {
        if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(get_logger(), "Action server waited too long");
            rclcpp::shutdown();
            return;
        }

        FollowPath::Goal goal;
        goal.path = controllers::TestPathGenerator(20).BuildPath();
        goal.path.header.frame_id = "map";
        goal.path.header.stamp = now();

        publisher_->publish(goal.path);

        auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();

        send_goal_options.goal_response_callback = std::bind(
            &ControllerTestClient::GoalResponseCallback,
            this,
            std::placeholders::_1);

        send_goal_options.feedback_callback = std::bind(
            &ControllerTestClient::FeedbackCallback, 
            this,
            std::placeholders::_1,
            std::placeholders::_2);

        send_goal_options.result_callback = std::bind(
            &ControllerTestClient::ResultCallback,
            this,
            std::placeholders::_1);

        client_->async_send_goal(goal, send_goal_options);
    }

private:
    rclcpp_action::Client<FollowPath>::SharedPtr client_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;

    void GoalResponseCallback(const GoalHandle::SharedPtr & goal_handle)
    {
        if (goal_handle) {
            RCLCPP_INFO(get_logger(), "Goal ACCEPTED");
        } else {
            RCLCPP_INFO(get_logger(), "Goal REJECTED");
        }
    }

    void FeedbackCallback(
        GoalHandle::SharedPtr, 
        const std::shared_ptr<const FollowPath::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "%f distance left to the goal", feedback->distance_to_goal);
    }

    void ResultCallback(const GoalHandle::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_INFO(this->get_logger(), "Goal aborted!");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(), "Goal cancelled!");
                break;
            case rclcpp_action::ResultCode::UNKNOWN:
                RCLCPP_INFO(this->get_logger(), "Goal unkown!");
                break;
        }
        rclcpp::shutdown();
    }
};

}  // namespace controllers
RCLCPP_COMPONENTS_REGISTER_NODE(controllers::ControllerTestClient)