#pragma once
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <QImage>
#include <QLabel>
#include <QTimer>

namespace ssb_rviz {
class LivePreviewPanel : public rviz_common::Panel {
  Q_OBJECT
 public:
  explicit LivePreviewPanel(QWidget* parent = nullptr);
  void onInitialize() override;
 Q_SIGNALS:
  void received(const QString& text);
  void missionReceived(const QString& text);
 protected:
  void resizeEvent(QResizeEvent* event) override;
 private:
  void showPreview(const QString& text);
  void showMission(const QString& text);
  void paintImage();
  QLabel *image_label_, *caption_;
  QImage image_;
  QString output_;
  QTimer* watchdog_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr preview_, status_;
};
}
