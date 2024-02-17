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
    worker_thread_ = std::thread(std::bind(&ClientNode::workerFunc, this));
  }

  ~ClientNode()
  {
    worker_thread_.join();
  }

private:
  void workerFunc(void)
  {
    if (!wait_client_->wait_for_action_server(2000ms)) {
      RCLCPP_WARN(this->get_logger(), "server not found");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = ros2_lecture_msgs::action::WaitTask::Goal();
    goal_msg.name = "aaa";
    goal_msg.duration = 10.0f;
    auto send_goal_options =
      rclcpp_action::Client<ros2_lecture_msgs::action::WaitTask>::SendGoalOptions();
    send_goal_options.feedback_callback = std::bind(&ClientNode::feedbackCallback, this, _1, _2);

    RCLCPP_INFO(this->get_logger(), "send goal");
    std::shared_future<rclcpp_action::ClientGoalHandle<ros2_lecture_msgs::action::WaitTask>::SharedPtr>
    goal_feature = wait_client_->async_send_goal(goal_msg, send_goal_options);
    rclcpp_action::ClientGoalHandle<ros2_lecture_msgs::action::WaitTask>::SharedPtr goal_handle =
      goal_feature.get();
    if (goal_handle) {
      printf("id ");
      for (const auto c : goal_handle->get_goal_id()) {
        printf("%x ", c);
      }
      printf("\n");
    } else {
      RCLCPP_WARN(this->get_logger(), "goal_handle empty");
      rclcpp::shutdown();
      return;
    }

    std::shared_future<rclcpp_action::ClientGoalHandle<ros2_lecture_msgs::action::WaitTask>::WrappedResult>
    result_feature = wait_client_->async_get_result(goal_handle);
    rclcpp_action::ClientGoalHandle<ros2_lecture_msgs::action::WaitTask>::WrappedResult
      wrapped_result = result_feature.get();
    RCLCPP_INFO(
      this->get_logger(), "result %d %s", (int)wrapped_result.code,
      wrapped_result.result->message.c_str());

    rclcpp::shutdown();
  }

  void feedbackCallback(
    rclcpp_action::ClientGoalHandle<ros2_lecture_msgs::action::WaitTask>::SharedPtr goal_handle,
    const std::shared_ptr<const ros2_lecture_msgs::action::WaitTask::Feedback> feedback)
  {
    (void) goal_handle;
    RCLCPP_INFO(this->get_logger(), "feedback %f", feedback->left_duration);
  }

  std::thread worker_thread_;
  rclcpp_action::Client<ros2_lecture_msgs::action::WaitTask>::SharedPtr wait_client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto client_node = std::make_shared<ClientNode>();
  rclcpp::spin(client_node);
  rclcpp::shutdown();
  return 0;
}
