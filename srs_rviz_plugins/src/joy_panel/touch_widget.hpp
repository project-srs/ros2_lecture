#pragma once
#include <QtWidgets>

class TouchWidget : public QWidget
{
  Q_OBJECT
public:
  TouchWidget(QWidget *parent = 0);

  // event
  void setEnabled(bool enable);
  void paintEvent(QPaintEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;

public Q_SLOTS:
  void setEnabled(int state);

Q_SIGNALS:
  void notifyScaledPoint(QPointF);
  void notifyScaledPoint(QString);

private:
  void setValue(QPointF point);
  int getPadSize(void) const;
  QPoint getCenter(void) const;
  QPointF getScaledPoint(const QPoint &point) const;
  QPoint getPadPoint(const QPointF &point) const;

  // property
  bool grayout_{false};
  QPointF latest_scaled_value_{};
};