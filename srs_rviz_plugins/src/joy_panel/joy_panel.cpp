#include "joy_panel.hpp"

#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>

#include <QPainter>
#include <QMouseEvent>
#include <QSizePolicy>

namespace srs_rviz_plugins
{
  JoyPanel::JoyPanel(QWidget *parent) : rviz_common::Panel(parent)
  {
    QVBoxLayout *layout = new QVBoxLayout;

    QHBoxLayout *layout_1st = new QHBoxLayout;
    enable_check_ = new QCheckBox("Topic");
    layout_1st->addWidget(enable_check_);
    topic_combo_ = new QComboBox();
    topic_combo_->setEditable(true);
    layout_1st->addWidget(topic_combo_);
    layout->addLayout(layout_1st);

    touch_ = new TouchWidget();
    layout->addWidget(touch_);

    QHBoxLayout *layout_3rd = new QHBoxLayout;
    a_button_ = new QPushButton("A");
    b_button_ = new QPushButton("B");
    layout_3rd->addWidget(a_button_);
    layout_3rd->addWidget(b_button_);
    layout->addLayout(layout_3rd);

    setLayout(layout);

    interval_timer_ = new QTimer(this);

    connect(interval_timer_, &QTimer::timeout, this, &JoyPanel::onTick);
    connect(enable_check_, &QCheckBox::stateChanged, this, &JoyPanel::onCheckChange);
    connect(a_button_, &QPushButton::clicked, this, &JoyPanel::onClickA);
    connect(b_button_, &QPushButton::clicked, this, &JoyPanel::onClickB);
    connect(touch_, qOverload<QPointF>(&TouchWidget::notifyScaledPoint), this, &JoyPanel::onTouchChange);

    interval_timer_->start(100);
    touch_->setEnabled(false);
    touch_->update();
  }

  void JoyPanel::onInitialize()
  {
    joy_handler_.setRosNodePtr(this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node());
    updateTopicList();
  }

  void JoyPanel::onCheckChange(int state)
  {
    if (state == Qt::Checked)
    {
      std::string topic_name = topic_combo_->currentText().toStdString();
      bool ret = joy_handler_.initializePublisher(topic_name);
      if (!ret)
      {
        return;
      }
      topic_combo_->setEnabled(false);
      touch_->setEnabled(true);
      is_active_ = true;
    }
    else
    {
      joy_handler_.finalizePublisher();
      topic_combo_->setEnabled(true);
      touch_->setEnabled(false);
      is_active_ = false;
      updateTopicList();
    }
  }

  void JoyPanel::onClickA()
  {
    a_clicked_ = true;
  }

  void JoyPanel::onClickB()
  {
    b_clicked_ = true;
  }

  void JoyPanel::onTick()
  {
    if (is_active_)
    {
      float axis0 = -touch_point_.y();
      float axis1 = -touch_point_.x();
      joy_handler_.publishJoy({axis0, axis1}, {a_clicked_, b_clicked_});
      a_clicked_ = false;
      b_clicked_ = false;
    }
  }

  void JoyPanel::save(rviz_common::Config config) const
  {
    rviz_common::Panel::save(config);
    config.mapSetValue("BaseTopic", topic_combo_->currentText());
    config.mapSetValue("Checked", enable_check_->isChecked());
  }

  void JoyPanel::load(const rviz_common::Config &config)
  {
    rviz_common::Panel::load(config);
    QString tmp_text;
    bool tmp_bool;
    if (config.mapGetString("BaseTopic", &tmp_text))
    {
      topic_combo_->setCurrentText(tmp_text);
    }
    if (config.mapGetBool("Checked", &tmp_bool))
    {
      enable_check_->setChecked(tmp_bool);
    }
  }

  void JoyPanel::updateTopicList(void)
  {
    std::string previous_topic_name = topic_combo_->currentText().toStdString();
    auto topic_list = joy_handler_.getTwistTopicList();
    topic_combo_->clear();
    int same_topic_index = -1;
    for (auto t : topic_list)
    {
      topic_combo_->addItem(t.c_str());
      if (t == previous_topic_name)
      {
        same_topic_index = topic_combo_->count() - 1;
      }
    }

    if (previous_topic_name != "")
    {
      if (same_topic_index < 0)
      {
        topic_combo_->addItem(previous_topic_name.c_str());
        same_topic_index = topic_combo_->count() - 1;
      }
      topic_combo_->setCurrentIndex(same_topic_index);
    }
  }

  void JoyPanel::onTouchChange(QPointF point)
  {
    touch_point_ = point;
  }

} // namespace srs_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(srs_rviz_plugins::JoyPanel, rviz_common::Panel)