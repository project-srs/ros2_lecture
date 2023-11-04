#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace srs_rviz_plugins
{
  class JoyHandler
  {
  public:
    JoyHandler(void) {}

    void setRosNodePtr(const rclcpp::Node::SharedPtr node_ptr)
    {
      node_ptr_ = node_ptr;
    }

    bool initializePublisher(const std::string topic_name)
    {
      if (topic_name == "")
      {
        return false;
      }
      joy_publisher_ = node_ptr_->create_publisher<sensor_msgs::msg::Joy>(topic_name, rclcpp::QoS(10));
      return true;
    }

    void finalizePublisher(void)
    {
      joy_publisher_.reset();
    }

    void publishJoy(std::vector<float> axes, std::vector<int> buttons)
    {
      sensor_msgs::msg::Joy msg{};
      msg.header.stamp = node_ptr_->now();
      msg.axes.resize(2);
      msg.axes = axes;
      msg.buttons = buttons;
      joy_publisher_->publish(msg);
    }

    std::vector<std::string> getTwistTopicList(void) const
    {
      return getTopicList("sensor_msgs/msg/Joy");
    }

  private:
    std::vector<std::string> getTopicList(const std::string type_name) const
    {
      std::map<std::string, std::vector<std::string>> topic_map = node_ptr_->get_topic_names_and_types();

      std::vector<std::string> output;
      for (auto pair : topic_map)
      {
        for (auto s : pair.second)
        {
          if (s == type_name)
          {
            output.push_back(pair.first);
            break;
          }
        }
      }
      return output;
    }

    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
  };

} // namespace srs_rviz_plugins
