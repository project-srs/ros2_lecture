#pragma once 

#include <QtWidgets>
#include <QComboBox>
#include "touch_widget.hpp"

#ifndef Q_MOC_RUN
#include "joy_handler.hpp"
#include <rviz_common/panel.hpp>
#endif

namespace srs_rviz_plugins
{

class JoyPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  JoyPanel(QWidget* parent = nullptr);
  void onInitialize() override;
  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;

public Q_SLOTS:
  void onCheckChange(int state);
  void onClickA(void);
  void onClickB(void);
  void onTick(void);
  void onTouchChange(QPointF point);

private:
  void updateTopicList(void);
  
  JoyHandler joy_handler_{};
  QCheckBox* enable_check_;
  QComboBox* topic_combo_;
  TouchWidget* touch_;
  QPushButton* a_button_;
  QPushButton* b_button_;
  QTimer* interval_timer_;
  bool is_active_{false};
  bool a_clicked_{false};
  bool b_clicked_{false};
  QPointF touch_point_{};
};

}  // namespace srs_rviz_plugins
