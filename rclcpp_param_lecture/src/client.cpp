#include <rclcpp/rclcpp.hpp>
#include <termios.h>

using namespace std::chrono_literals;
using namespace std::placeholders;

class ParamClient : public rclcpp::Node
{
public:
  ParamClient()
  : Node("param_client")
  {
    param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "param_server");
    parameter_sub_ =
      param_client_->on_parameter_event(
      std::bind(
        &ParamClient::ParameterEventCallback, this,
        std::placeholders::_1));
    interval_timer_ =
      this->create_wall_timer(1000ms, std::bind(&ParamClient::onIntervalTimer, this));
  }

private:
  void onIntervalTimer()
  {
    static int counter = 0;
    if (counter == 5) {
      if (!param_client_->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "service not ready");
      } else {
        param_client_->set_parameters({rclcpp::Parameter("int_param", 5)});
        RCLCPP_INFO(this->get_logger(), "set param: %s %d", "int_param", 5);
      }
    } else if (counter == 10) {
      if (!param_client_->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "service not ready");
      } else {
        param_client_->set_parameters({rclcpp::Parameter("int_param", -1)});
        RCLCPP_INFO(this->get_logger(), "set param: %s %d", "int_param", 2);
      }
    }
    counter++;
  }

  void ParameterEventCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "node: %s", msg->node.c_str());
    for (const auto & p : msg->changed_parameters) {
      RCLCPP_INFO(this->get_logger(), "change: %s", p.name.c_str());
    }
  }

  rclcpp::TimerBase::SharedPtr interval_timer_{nullptr};
  rclcpp::AsyncParametersClient::SharedPtr param_client_{nullptr};
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_sub_{nullptr};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto param_server = std::make_shared<ParamClient>();
  rclcpp::spin(param_server);
  rclcpp::shutdown();
  return 0;
}
