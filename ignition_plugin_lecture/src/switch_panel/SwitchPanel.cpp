#include "SwitchPanel.hpp"

#include <ignition/msgs/float.pb.h>
#include <ignition/plugin/Register.hh>

namespace iginition_plugin_lecture
{

SwitchPanel::SwitchPanel() : Plugin()
{
  CreateIgnitionIf();
}

SwitchPanel::~SwitchPanel()
{
}

void SwitchPanel::LoadConfig(const tinyxml2::XMLElement * _pluginElem)
{
  if (!_pluginElem) {
    return;
  }

  auto forward_speed_elem = _pluginElem->FirstChildElement("forward_speed");
  if (nullptr != forward_speed_elem)
  {
    forward_speed_ = forward_speed_elem->FloatText();
  }

  auto reverse_speed_elem = _pluginElem->FirstChildElement("reverse_speed");
  if (nullptr != reverse_speed_elem)
  {
    reverse_speed_ = reverse_speed_elem->FloatText();
  }
}

void SwitchPanel::OnForwardButton(void) {
  ignition::msgs::Float float_msg;
  float_msg.set_data(forward_speed_);
  speed_pub_.Publish(float_msg);
}

void SwitchPanel::OnStopButton(void) {
  ignition::msgs::Float float_msg;
  float_msg.set_data(0.0f);
  speed_pub_.Publish(float_msg);
}

void SwitchPanel::OnReverseButton(void) {
  ignition::msgs::Float float_msg;
  float_msg.set_data(reverse_speed_);
  speed_pub_.Publish(float_msg);
}

void SwitchPanel::CreateIgnitionIf(void){
  this->speed_pub_ = this->node_.Advertise<ignition::msgs::Float>("target_speed");
}

}

// Register this plugin
IGNITION_ADD_PLUGIN(iginition_plugin_lecture::SwitchPanel, ignition::gui::Plugin)
