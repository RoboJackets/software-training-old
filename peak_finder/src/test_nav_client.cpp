#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

namespace peak_finder
{

class TestNavClient : public rclcpp::Node
{
public:
    using Client = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>;

    explicit TestNavClient(const rclcpp::NodeOptions& options) : rclcpp::Node("test_nav_client", options)
    {
        client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/navigate_to_pose");
        timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&TestNavClient::sendGoal, this));
        RCLCPP_INFO(get_logger(), "Constructor done!");
    }

    ~TestNavClient() {
        if(running_)
        {
            client_->async_cancel_goal(goal_handle_);
        }
    }

private:
    Client::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    Client::GoalHandle::SharedPtr goal_handle_;
    bool running_ = false;

    void sendGoal()
    {
        RCLCPP_INFO(get_logger(), "sendGoal()");
        timer_->cancel();

        RCLCPP_INFO(get_logger(), "Waiting for server...");
        client_->wait_for_action_server();
        RCLCPP_INFO(get_logger(), "Server ready!");

        nav2_msgs::action::NavigateToPose::Goal goal;
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = now();
        goal.pose.pose.position.x = 0.2;

        Client::SendGoalOptions goal_options;
        goal_options.goal_response_callback = std::bind(&TestNavClient::goalResponseCallback, this, std::placeholders::_1);
        goal_options.feedback_callback = std::bind(&TestNavClient::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        goal_options.result_callback = std::bind(&TestNavClient::resultCallback, this, std::placeholders::_1);

        client_->async_send_goal(goal, goal_options);
        RCLCPP_INFO(get_logger(), "sendGoal() done!");
    }

    void goalResponseCallback(std::shared_future<Client::GoalHandle::SharedPtr> future)
     {
       goal_handle_ = future.get();
       if (!goal_handle_) {
         RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
         rclcpp::shutdown();
       } else {
         RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
         running_ = true;
       }
     }

    void feedbackCallback(Client::GoalHandle::SharedPtr goal_handle, const nav2_msgs::action::NavigateToPose::Feedback::ConstSharedPtr feedback)
    {
        RCLCPP_INFO(get_logger(), "Feedback received! Distance to goal: %f", feedback->distance_remaining);
    }

    void resultCallback(const Client::GoalHandle::WrappedResult& wrapped_result)
    {
        if(wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(get_logger(), "Succeeded!");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed!");
        }
        running_ = false;
        rclcpp::shutdown();
    }

};

}

RCLCPP_COMPONENTS_REGISTER_NODE(peak_finder::TestNavClient)
