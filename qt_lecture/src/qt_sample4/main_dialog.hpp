#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>

class MainDialog : public QDialog
{
  Q_OBJECT

public:
  MainDialog(QWidget * parent);

public Q_SLOTS:
  void setLabelText();
  void clearButtonText();

Q_SIGNALS:
  void modifyText(QString);

private:
  QLabel * label_;
  QLineEdit * lineEdit_;
  QPushButton * setButton_;
  QTimer * timer_;
};
