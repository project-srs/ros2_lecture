#include <ignition/gui/qt.h>
#include <ignition/transport/Node.hh>
#include <ignition/gui/Plugin.hh>

namespace iginition_plugin_lecture
{

class SwitchPanel : public ignition::gui::Plugin
{
  Q_OBJECT

public:
  SwitchPanel();
  virtual ~SwitchPanel();
  void LoadConfig(const tinyxml2::XMLElement * _pluginElem) override;

protected slots:
  void OnForwardButton(void);
  void OnStopButton(void);
  void OnReverseButton(void);

private:
  void CreateIgnitionIf(void);

private:
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher speed_pub_;
  float forward_speed_{1.0f};
  float reverse_speed_{1.0f};
};

}  // namespace iginition_plugin_lecture
