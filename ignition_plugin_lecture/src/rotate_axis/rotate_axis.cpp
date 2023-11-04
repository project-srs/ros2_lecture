
#include "rotate_axis.hpp"
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/msgs/float.pb.h>

namespace iginition_plugin_lecture
{

RotateAxis::RotateAxis()
{
  CreateIgnitionIf();
}

RotateAxis::~RotateAxis()
{
}

void RotateAxis::Configure(const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &_eventMgr)
{
  (void)_ecm;
  (void)_eventMgr;
  model_ = ignition::gazebo::Model(_entity);

  auto ptr = const_cast<sdf::Element *>(_sdf.get());
  sdf::ElementPtr target_joint_elem = ptr->GetElement("target_joint");
  if (target_joint_elem) {
    target_joint_name_ = target_joint_elem->Get<std::string>();
  } else {
    ignerr << "sdf target_joint not found" << std::endl;
  }
}

void RotateAxis::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
    (void)_info;
    (void)_ecm;
    ignition::gazebo::Entity joint = model_.JointByName(_ecm, target_joint_name_);
    if (joint == ignition::gazebo::kNullEntity){
      ignerr << target_joint_name_ <<" not found" << std::endl;
      return;
    }

    auto vel = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(joint);
    if (vel != nullptr)
    {
      *vel = ignition::gazebo::components::JointVelocityCmd({target_speed_});
    }
    else {
      _ecm.CreateComponent(joint, ignition::gazebo::components::JointVelocityCmd({target_speed_}));
    }
 }

void RotateAxis::Update(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  (void)_info;
  (void)_ecm;
}

void RotateAxis::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  (void)_info;
  (void)_ecm;
}

void RotateAxis::CreateIgnitionIf(void){
  this->node_.Subscribe("target_speed", &RotateAxis::OnSpeedMessage, this);
}

void RotateAxis::OnSpeedMessage(const ignition::msgs::Float & msg)
{
  target_speed_ = msg.data();
}

}

#include <ignition/plugin/Register.hh>
IGNITION_ADD_PLUGIN(
    iginition_plugin_lecture::RotateAxis,
    ignition::gazebo::System,
    iginition_plugin_lecture::RotateAxis::ISystemConfigure,
    iginition_plugin_lecture::RotateAxis::ISystemPreUpdate,
    iginition_plugin_lecture::RotateAxis::ISystemUpdate,
    iginition_plugin_lecture::RotateAxis::ISystemPostUpdate)
