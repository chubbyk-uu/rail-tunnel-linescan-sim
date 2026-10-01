#include "mission_panel.hpp"
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_rendering/render_window.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QUuid>
#include <QVBoxLayout>
#include <QDir>
#include <QFile>
#include <QRegularExpression>

namespace ssb_rviz {
MissionPanel::MissionPanel(QWidget* parent) : Panel(parent) {
  auto* layout = new QVBoxLayout(this);
  auto* form = new QFormLayout;
  start_ = new QDoubleSpinBox; distance_ = new QDoubleSpinBox;
  start_->setRange(0, 20); start_->setValue(3); start_->setDecimals(3);
  distance_->setRange(.12, 20); distance_->setValue(3); distance_->setDecimals(3);
  start_->setSuffix(" m"); distance_->setSuffix(" m");
  start_->setObjectName("mission_start"); distance_->setObjectName("mission_distance");
  form->addRow("任务起点", start_); form->addRow("前进距离", distance_); layout->addLayout(form);
  extent_ = new QLabel; extent_->setWordWrap(true); layout->addWidget(extent_);
  connect(start_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this]{updateExtent();});
  connect(distance_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this]{updateExtent();});
  updateExtent();
  auto* row = new QHBoxLayout;
  begin_ = new QPushButton("开始"); pause_ = new QPushButton("暂停");
  resume_ = new QPushButton("继续"); stop_ = new QPushButton("结束任务");
  begin_->setShortcut(QKeySequence("Ctrl+Alt+S")); pause_->setShortcut(QKeySequence("Ctrl+Alt+P"));
  resume_->setShortcut(QKeySequence("Ctrl+Alt+R")); stop_->setShortcut(QKeySequence("Ctrl+Alt+E"));
  begin_->setToolTip("开始任务 (Ctrl+Alt+S)"); pause_->setToolTip("暂停 (Ctrl+Alt+P)");
  resume_->setToolTip("继续 (Ctrl+Alt+R)"); stop_->setToolTip("结束并保存 (Ctrl+Alt+E)");
  for (auto* button : {begin_, pause_, resume_, stop_}) row->addWidget(button);
  begin_->setObjectName("mission_begin"); pause_->setObjectName("mission_pause");
  resume_->setObjectName("mission_resume"); stop_->setObjectName("mission_stop");
  layout->addLayout(row);
  connect(begin_, &QPushButton::clicked, this, [this]{send("start");});
  connect(pause_, &QPushButton::clicked, this, [this]{send("pause");});
  connect(resume_, &QPushButton::clicked, this, [this]{send("resume");});
  connect(stop_, &QPushButton::clicked, this, [this]{send("stop");});
  status_ = new QLabel("等待任务管理器"); progress_ = new QLabel;
  output_ = new QLabel; output_->setWordWrap(true); output_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  error_ = new QLabel; error_->setWordWrap(true); error_->setStyleSheet("color: #e87070");
  layout->addWidget(status_); layout->addWidget(progress_); layout->addWidget(output_); layout->addWidget(error_);
  auto* note = new QLabel("场景/车辆：仿真真值，仅供观察。\n任务距离和扫描控制：后轮编码器估计。\n暂停冻结仿真，已有图像继续落盘。");
  note->setWordWrap(true); layout->addWidget(note); layout->addStretch();
  connect(this, &MissionPanel::received, this, &MissionPanel::showStatus, Qt::QueuedConnection);
  watchdog_ = new QTimer(this); watchdog_->setSingleShot(true); watchdog_->setInterval(3000);
  connect(watchdog_, &QTimer::timeout, this, [this] {
    connected_ = false; status_->setText("任务管理器连接中断");
    for (auto* b : {begin_, pause_, resume_, stop_}) b->setEnabled(false);
  });
  for (auto* b : {begin_, pause_, resume_, stop_}) b->setEnabled(false);
}

void MissionPanel::onInitialize() {
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  commands_ = node->create_publisher<std_msgs::msg::String>("/ssb/mission/command", 10);
  auto qos = rclcpp::QoS(1).transient_local();
  subscription_ = node->create_subscription<std_msgs::msg::String>("/ssb/mission/status", qos,
    [this](std_msgs::msg::String::ConstSharedPtr msg){Q_EMIT received(QString::fromStdString(msg->data));});
  // Opt-in local review: read the render buffer directly (WSLg X11 grabs are black).
  review_dir_ = qEnvironmentVariable("SSB_RVIZ_REVIEW_DIR");
  if (!review_dir_.isEmpty()) {
    connect(this, &MissionPanel::inspectionRequested, this, &MissionPanel::saveReview, Qt::QueuedConnection);
    review_subscription_ = node->create_subscription<std_msgs::msg::String>("/ssb/mission/review", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg){Q_EMIT inspectionRequested(QString::fromStdString(msg->data));});
    QTimer::singleShot(5000, this, [this]{saveReview("");});
  }
}

void MissionPanel::saveReview(const QString& name) {
  if (!name.isEmpty() && !QRegularExpression("^[A-Za-z0-9_-]{1,32}$").match(name).hasMatch()) return;
  QDir().mkpath(review_dir_);
  const QString prefix = review_dir_+"/"+(name.isEmpty() ? "" : name+"_");
  window()->grab().save(prefix+"panel.png");
  getDisplayContext()->getViewManager()->getRenderPanel()->getRenderWindow()
    ->captureScreenShot((prefix+"scene.png").toStdString());
  QJsonObject result{{"connected", connected_}, {"begin_enabled", begin_->isEnabled()},
    {"pause_enabled", pause_->isEnabled()}, {"resume_enabled", resume_->isEnabled()},
    {"stop_enabled", stop_->isEnabled()}, {"status", status_->text()},
    {"state", last_state_}, {"pending", pending_}};
  QFile file(prefix+"panel.json");
  if (file.open(QIODevice::WriteOnly)) file.write(QJsonDocument(result).toJson());
}

void MissionPanel::send(const QString& action) {
  if (!connected_ || !pending_.isEmpty()) return;
  pending_ = QUuid::createUuid().toString(QUuid::WithoutBraces);
  QJsonObject object{{"id", pending_}, {"action", action}};
  if (action == "start") { object["start_m"] = start_->value(); object["distance_m"] = distance_->value(); }
  std_msgs::msg::String msg; msg.data = QJsonDocument(object).toJson(QJsonDocument::Compact).toStdString();
  commands_->publish(msg);
  for (auto* b : {begin_, pause_, resume_, stop_}) b->setEnabled(false);
}

void MissionPanel::showStatus(const QString& text) {
  const auto doc = QJsonDocument::fromJson(text.toUtf8()); if (!doc.isObject()) return;
  const auto object = doc.object(); const QString state = object["state"].toString();
  last_state_ = state;
  connected_ = true; watchdog_->start();
  pitch_ = object["scan_pitch_m"].toDouble(); updateExtent();
  if (object["command_result"].toObject()["id"].toString() == pending_) pending_.clear();
  const bool terminal = state == "idle" || state == "complete" || state == "stopped" || state == "failed";
  const bool available = pending_.isEmpty();
  start_->setEnabled(terminal && available); distance_->setEnabled(terminal && available);
  begin_->setEnabled(terminal && available); pause_->setEnabled(state == "running" && available);
  resume_->setEnabled(state == "paused" && available);
  stop_->setEnabled((state == "running" || state == "paused") && available);
  const QJsonObject labels{{"idle", "就绪"}, {"starting", "初始化"}, {"running", "运行"},
    {"paused", "已暂停"}, {"draining", "运动结束，等待图像落盘"},
    {"complete", "采集完成，校正留待拼接前"}, {"stopped", "提前结束，原始数据已保存"}, {"failed", "失败"}};
  status_->setText("状态："+labels[state].toString(state)+(object["dynamics_only"].toBool() ? "（仅动力学，不采图）" : ""));
  const double travelled = object["distance_estimated_m"].toDouble();
  const double distance = state == "idle" ? distance_->value() : object["task"].toObject()["distance_m"].toDouble();
  progress_->setText(QString("估计行驶：%1 m；剩余：%2 m\n速度：%3 m/s；扫描：%4 rad/s\n行数：生成 %5 / 已保存 %6\n成像滞后：%7 s\n扫描角度：%8 rad")
    .arg(travelled,0,'f',3).arg(std::max(0.,distance-travelled),0,'f',3)
    .arg(object["speed_m_s"].toDouble(),0,'f',3).arg(object["scan_rate_rad_s"].toDouble(),0,'f',3)
    .arg(object["rows_generated"].toDouble(),0,'f',0).arg(object["rows_saved"].toDouble(),0,'f',0)
    .arg(object["imaging_lag_s"].toDouble(),0,'f',3).arg(object["scan_rad"].toDouble(),0,'f',3));
  output_->setText("输出："+object["output"].toString()); error_->setText(object["error"].toString());
}

void MissionPanel::updateExtent() {
  const double start = start_->value(), end = start+distance_->value();
  QString coverage = "完整内壁覆盖：等待配置";
  if (pitch_ > 0) {
    coverage = distance_->value() > 2*pitch_
      ? QString("全角度覆盖保守估计：%1–%2 m").arg(start+pitch_,0,'f',3).arg(end-pitch_,0,'f',3)
      : "当前短行程无法给出全角度覆盖保守区间";
  }
  extent_->setText(QString("预计车体终点：%1 m\n%2\n完整覆盖须采集后核验。").arg(end,0,'f',3).arg(coverage));
}
}
PLUGINLIB_EXPORT_CLASS(ssb_rviz::MissionPanel, rviz_common::Panel)
