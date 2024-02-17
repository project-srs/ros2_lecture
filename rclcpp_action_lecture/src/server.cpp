#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros2_lecture_msgs/action/wait_task.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

class ServerNode : public rclcpp::Node
{
public:
  ServerNode()
  : Node("server_node")
  {
    wait_server_ =
      rclcpp_action::create_server<ros2_lecture_msgs::action::WaitTask>(
      this, "~/task", std::bind(&ServerNode::handleGoal, this, _1, _2),
      std::bind(&ServerNode::handleCancel, this, _1),
      std::bind(&ServerNode::handleAccepted, this, _1));
    interval_timer_ =
      this->create_wall_timer(1000ms, std::bind(&ServerNode::onIntervalTimer, this));
  }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ros2_lecture_msgs::action::WaitTask::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "handleGoal %s", goal->name.c_str());
    if (!goal_handle_) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ros2_lecture_msgs::action::WaitTask>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "cancel %s", goal_handle->get_goal()->name.c_str());
    goal_handle_.reset();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ros2_lecture_msgs::action::WaitTask>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "accept %s", goal_handle->get_goal()->name.c_str());
    goal_handle_ = goal_handle;
    counter_ = 0;
  }

  void onIntervalTimer(void)
  {
    if (goal_handle_) {
      counter_++;
      const int threshold_count = goal_handle_->get_goal()->duration * 1.0f;
      RCLCPP_INFO(this->get_logger(), "%d <= %d", threshold_count, counter_);
      if (threshold_count <= counter_) {
        RCLCPP_INFO(this->get_logger(), "success %s", goal_handle_->get_goal()->name.c_str());
        auto result = std::make_shared<ros2_lecture_msgs::action::WaitTask::Result>();
        result->message = "full count";
        goal_handle_->succeed(result);
        goal_handle_.reset();
      } else {
        auto feedback = std::make_shared<ros2_lecture_msgs::action::WaitTask::Feedback>();
        feedback->left_duration = (threshold_count - counter_) * 1.0f;
        goal_handle_->publish_feedback(feedback);
      }
    }
  }

  rclcpp_action::Server<ros2_lecture_msgs::action::WaitTask>::SharedPtr wait_server_;
  rclcpp::TimerBase::SharedPtr interval_timer_{nullptr};
  int counter_{0};

  std::shared_ptr<rclcpp_action::ServerGoalHandle<ros2_lecture_msgs::action::WaitTask>> goal_handle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto server_node = std::make_shared<ServerNode>();
  rclcpp::spin(server_node);
  rclcpp::shutdown();
  return 0;
}
