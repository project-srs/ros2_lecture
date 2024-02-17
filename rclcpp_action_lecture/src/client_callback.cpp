#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros2_lecture_msgs/action/wait_task.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

class ClientNode : public rclcpp::Node
{
public:
  ClientNode()
  : Node("client_node")
  {
    wait_client_ = rclcpp_action::create_client<ros2_lecture_msgs::action::WaitTask>(
      this,
      "/server_node/task");
    interval_timer_ =
      this->create_wall_timer(1000ms, std::bind(&ClientNode::onIntervalTimer, this));

  }

private:
  void onIntervalTimer(void)
  {
    if (!send_goal_) {
      send_goal_ = true;
      if (!wait_client_->wait_for_action_server(2000ms)) {
        RCLCPP_WARN(this->get_logger(), "server not found");
        rclcpp::shutdown();
        return;
      }

      auto goal_msg = ros2_lecture_msgs::action::WaitTask::Goal();
      goal_msg.name = "bbb";
      goal_msg.duration = 9.0f;
      auto send_goal_options =
        rclcpp_action::Client<ros2_lecture_msgs::action::WaitTask>::SendGoalOptions();
      send_goal_options.goal_response_callback = std::bind(
        &ClientNode::goalResponseCallback, this,
        _1);
      send_goal_options.feedback_callback = std::bind(&ClientNode::feedbackCallback, this, _1, _2);
      send_goal_options.result_callback = std::bind(&ClientNode::resultCallback, this, _1);

      RCLCPP_INFO(this->get_logger(), "send goal");
      wait_client_->async_send_goal(goal_msg, send_goal_options);
    }
  }

  void goalResponseCallback(
    rclcpp_action::ClientGoalHandle<ros2_lecture_msgs::action::WaitTask>::SharedPtr goal_handle)
  {
    if (goal_handle) {
      RCLCPP_INFO(this->get_logger(), "get goal_handle");
    } else {
      RCLCPP_WARN(this->get_logger(), "empty goal_handle");
    }
  }

  void feedbackCallback(
    rclcpp_action::ClientGoalHandle<ros2_lecture_msgs::action::WaitTask>::SharedPtr goal_handle,
    const std::shared_ptr<const ros2_lecture_msgs::action::WaitTask::Feedback> feedback)
  {
    (void) goal_handle;
    RCLCPP_INFO(this->get_logger(), "feedback %f", feedback->left_duration);
  }

  void resultCallback(
    const rclcpp_action::ClientGoalHandle<ros2_lecture_msgs::action::WaitTask>::WrappedResult result)
  {
    RCLCPP_INFO(
      this->get_logger(), "result %d %s", (int)result.code,
      result.result->message.c_str());
    rclcpp::shutdown();
  }

  rclcpp_action::Client<ros2_lecture_msgs::action::WaitTask>::SharedPtr wait_client_;
  rclcpp::TimerBase::SharedPtr interval_timer_{nullptr};
  bool send_goal_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto client_node = std::make_shared<ClientNode>();
  rclcpp::spin(client_node);
  rclcpp::shutdown();
  return 0;
}
