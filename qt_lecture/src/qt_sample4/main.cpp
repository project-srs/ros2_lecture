#include <QApplication>
#include "main_dialog.hpp"

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  QWidget *window = new QWidget;
  MainDialog *dialog = new MainDialog(window);
  dialog->show();
  return app.exec();
}