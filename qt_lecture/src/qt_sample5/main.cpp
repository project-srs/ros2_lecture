#include <QApplication>
#include <QVBoxLayout>
#include <QLabel>
#include <QCheckBox>
#include "touch_widget.hpp"

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);

  QWidget *window = new QWidget;
  QVBoxLayout *layout = new QVBoxLayout;
  TouchWidget *touch_widget = new TouchWidget();
  touch_widget->setEnabled(false);
  QCheckBox *check = new QCheckBox("enable");
  QLabel *label = new QLabel("");

  layout->addWidget(touch_widget);
  layout->addWidget(label);
  layout->addWidget(check);
  window->setLayout(layout);

  QObject::connect(check, &QCheckBox::stateChanged, touch_widget, qOverload<int>(&TouchWidget::setEnabled));
  QObject::connect(touch_widget, qOverload<QString>(&TouchWidget::notifyScaledPoint), label, &QLabel::setText);

  window->show();
  return app.exec();
}