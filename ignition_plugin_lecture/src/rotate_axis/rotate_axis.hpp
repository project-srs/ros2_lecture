#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/transport/Node.hh>

namespace iginition_plugin_lecture
{
  class RotateAxis:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate,
    public ignition::gazebo::ISystemUpdate,
    public ignition::gazebo::ISystemPostUpdate
  {
    public: 
        RotateAxis();
        ~RotateAxis() override;
        void Configure(const ignition::gazebo::Entity &_entity,
            const std::shared_ptr<const sdf::Element> &_sdf,
            ignition::gazebo::EntityComponentManager &_ecm,
            ignition::gazebo::EventManager &_eventMgr) override;

        void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;
        void Update(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;
        void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;
    private:
      void CreateIgnitionIf(void);
      void OnSpeedMessage(const ignition::msgs::Float & msg);

      ignition::gazebo::Model model_;
      ignition::transport::Node node_;
      std::string target_joint_name_{""};
      float target_speed_{0.0f};
  };
}