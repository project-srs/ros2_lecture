#include <QApplication>
#include <QLabel>

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  QLabel *label = new QLabel("######## basic1 ########");
  label->show();
  return app.exec();
  return 0;
}