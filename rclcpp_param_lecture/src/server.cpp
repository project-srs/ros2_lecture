#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

class ParamServer : public rclcpp::Node
{
public:
  ParamServer()
  : Node("param_server")
  {
    this->declare_parameter("int_param", 1);
    parameter_handle_ =
      this->add_on_set_parameters_callback(
      std::bind(
        &ParamServer::parametersCallback, this,
        std::placeholders::_1));
    interval_timer_ =
      this->create_wall_timer(2000ms, std::bind(&ParamServer::onIntervalTimer, this));
  }

private:
  void onIntervalTimer()
  {
    int param = this->get_parameter("int_param").as_int();
    RCLCPP_INFO(this->get_logger(), "value %d", param);
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "int_param" &&
        parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        int param = parameter.as_int();
        if (0 <= param) {
          RCLCPP_INFO(this->get_logger(), "changed: %s %d", parameter.get_name().c_str(), param);
        } else {
          RCLCPP_INFO(this->get_logger(), "reject: %s %d", parameter.get_name().c_str(), param);
          result.successful = false;
          result.reason = "negative value";
          return result;
        }
      }
    }
    return result;
  }

private:
  rclcpp::TimerBase::SharedPtr interval_timer_{nullptr};
  OnSetParametersCallbackHandle::SharedPtr parameter_handle_{nullptr};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto param_server = std::make_shared<ParamServer>();
  rclcpp::spin(param_server);
  rclcpp::shutdown();
  return 0;
}
