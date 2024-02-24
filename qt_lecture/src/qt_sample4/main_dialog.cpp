#include <QVBoxLayout>

#include "main_dialog.hpp"

MainDialog::MainDialog(QWidget * parent)
: QDialog(parent)
{
  label_ = new QLabel(tr("empty"));
  setButton_ = new QPushButton(tr("Set"));
  lineEdit_ = new QLineEdit;
  timer_ = new QTimer;

  connect(setButton_, &QPushButton::clicked, this, &MainDialog::setLabelText);
  connect(timer_, &QTimer::timeout, this, &MainDialog::clearButtonText);
  connect(this, &MainDialog::modifyText, label_, &QLabel::setText);

  QVBoxLayout * layout = new QVBoxLayout;
  layout->addWidget(label_);
  layout->addWidget(lineEdit_);
  layout->addWidget(setButton_);
  setLayout(layout);
}

void MainDialog::setLabelText()
{
  QString text = lineEdit_->text();
  setButton_->setText("Done");
  timer_->start(500);
  Q_EMIT modifyText(text);
}

void MainDialog::clearButtonText()
{
  setButton_->setText("Set");
}
