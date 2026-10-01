#pragma once
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QTimer>

namespace ssb_rviz {
class MissionPanel : public rviz_common::Panel {
  Q_OBJECT
 public:
  explicit MissionPanel(QWidget* parent = nullptr);
  void onInitialize() override;
 Q_SIGNALS:
  void received(const QString& text);
  void inspectionRequested(const QString& name);
 private:
  void send(const QString& action);
  void showStatus(const QString& text);
  void updateExtent();
  void updateControls();
  void saveReview(const QString& name);
  QDoubleSpinBox *start_, *distance_;
  QLabel *extent_, *status_, *progress_, *output_, *error_;
  QPushButton *begin_, *pause_, *resume_, *stop_;
  QTimer *watchdog_, *command_watchdog_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr commands_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr review_subscription_;
  QString pending_, command_error_;
  bool connected_ = false;
  double pitch_ = 0.;
  QString last_state_, review_dir_;
};
}
