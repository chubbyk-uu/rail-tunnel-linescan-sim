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
#include <QDockWidget>
#include <QMainWindow>

namespace ssb_rviz {
MissionPanel::MissionPanel(QWidget* parent) : Panel(parent) {
  auto* layout = new QVBoxLayout(this);
  auto* form = new QFormLayout;
  mode_ = new QComboBox; mode_->setObjectName("mission_mode");
  mode_->addItem("Vehicle travel", "travel"); mode_->addItem("Wall coverage", "wall");
  form->addRow("Task mode", mode_);
  start_ = new QDoubleSpinBox; distance_ = new QDoubleSpinBox;
  start_->setRange(0, 20); start_->setValue(3); start_->setDecimals(3);
  distance_->setRange(1, 20); distance_->setValue(3); distance_->setDecimals(3);
  start_->setSuffix(" m"); distance_->setSuffix(" m");
  start_->setToolTip("Vehicle is placed here when you click Start.");
  start_->setObjectName("mission_start"); distance_->setObjectName("mission_distance");
  start_label_ = new QLabel("Start position"); distance_label_ = new QLabel("Travel distance");
  form->addRow(start_label_, start_); form->addRow(distance_label_, distance_); layout->addLayout(form);
  extent_ = new QLabel; extent_->setWordWrap(true); layout->addWidget(extent_);
  connect(start_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this]{updateExtent();});
  connect(distance_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this]{updateExtent();});
  connect(mode_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]{updateExtent();});
  updateExtent();
  auto* row = new QHBoxLayout;
  begin_ = new QPushButton("Start"); pause_ = new QPushButton("Pause");
  resume_ = new QPushButton("Resume"); stop_ = new QPushButton("Stop");
  begin_->setShortcut(QKeySequence("Ctrl+Alt+S")); pause_->setShortcut(QKeySequence("Ctrl+Alt+P"));
  resume_->setShortcut(QKeySequence("Ctrl+Alt+R")); stop_->setShortcut(QKeySequence("Ctrl+Alt+E"));
  begin_->setToolTip("Start mission (Ctrl+Alt+S)"); pause_->setToolTip("Pause (Ctrl+Alt+P)");
  resume_->setToolTip("Resume (Ctrl+Alt+R)"); stop_->setToolTip("Stop and save (Ctrl+Alt+E)");
  for (auto* button : {begin_, pause_, resume_, stop_}) row->addWidget(button);
  begin_->setObjectName("mission_begin"); pause_->setObjectName("mission_pause");
  resume_->setObjectName("mission_resume"); stop_->setObjectName("mission_stop");
  layout->addLayout(row);
  connect(begin_, &QPushButton::clicked, this, [this]{send("start");});
  connect(pause_, &QPushButton::clicked, this, [this]{send("pause");});
  connect(resume_, &QPushButton::clicked, this, [this]{send("resume");});
  connect(stop_, &QPushButton::clicked, this, [this]{send("stop");});
  status_ = new QLabel("Waiting for mission manager"); progress_ = new QLabel;
  status_->setWordWrap(true); progress_->setWordWrap(true);
  output_ = new QLabel; output_->setWordWrap(true); output_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  error_ = new QLabel; error_->setWordWrap(true); error_->setStyleSheet("color: #e87070");
  layout->addWidget(status_); layout->addWidget(progress_); layout->addWidget(output_); layout->addWidget(error_);
  auto* note = new QLabel("Scene: simulated pose, for visualization only.\nTravel and scan: measuring-wheel odometry.\nPause freezes simulation; pending images are saved.");
  note->setWordWrap(true); layout->addWidget(note); layout->addStretch();
  connect(this, &MissionPanel::received, this, &MissionPanel::showStatus, Qt::QueuedConnection);
  command_watchdog_ = new QTimer(this);
  command_watchdog_->setTimerType(Qt::PreciseTimer);
  command_watchdog_->setSingleShot(true); command_watchdog_->setInterval(90000);
  connect(command_watchdog_, &QTimer::timeout, this, [this] {
    if (pending_.isEmpty()) return;
    pending_.clear();
    command_error_ = "Command acknowledgement timed out. Check task state before trying again.";
    error_->setText(command_error_); updateControls();
  });
  watchdog_ = new QTimer(this); watchdog_->setSingleShot(true); watchdog_->setInterval(3000);
  connect(watchdog_, &QTimer::timeout, this, [this] {
    connected_ = false; status_->setText("Mission manager disconnected");
    if (!pending_.isEmpty()) {
      pending_.clear(); command_watchdog_->stop();
      command_error_ = "Connection lost while awaiting acknowledgement. Check task state after reconnecting.";
      error_->setText(command_error_);
    }
    updateControls();
  });
  for (auto* b : {begin_, pause_, resume_, stop_}) b->setEnabled(false);
}

void MissionPanel::onInitialize() {
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  commands_ = node->create_publisher<std_msgs::msg::String>("/ssb/mission/command", 10);
  auto qos = rclcpp::QoS(1).transient_local();
  subscription_ = node->create_subscription<std_msgs::msg::String>("/ssb/mission/status", qos,
    [this](std_msgs::msg::String::ConstSharedPtr msg){Q_EMIT received(QString::fromStdString(msg->data));});
  QTimer::singleShot(0, this, [this] {
    QWidget* parent = parentWidget();
    while (parent && !qobject_cast<QDockWidget*>(parent)) parent = parent->parentWidget();
    auto* main = qobject_cast<QMainWindow*>(window());
    if (main && parent) main->addDockWidget(Qt::RightDockWidgetArea, qobject_cast<QDockWidget*>(parent));
  });
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
    {"state", last_state_}, {"pending", pending_}, {"error", error_->text()},
    {"command_timeout_ms", command_watchdog_->interval()},
    {"command_timer_active", command_watchdog_->isActive()},
    {"task_mode", mode_->currentData().toString()}, {"extent", extent_->text()}};
  const auto mode_center = mode_->mapToGlobal(mode_->rect().center());
  result["mode_center_x"] = mode_center.x(); result["mode_center_y"] = mode_center.y();
  const auto begin_center = begin_->mapToGlobal(begin_->rect().center());
  result["begin_center_x"] = begin_center.x(); result["begin_center_y"] = begin_center.y();
  if (auto* preview = window()->findChild<QLabel*>("raw_preview_image")) {
    result["preview_ready"] = preview->property("preview_ready").toBool();
    result["preview_output"] = preview->property("preview_output").toString();
    result["preview_first_row"] = preview->property("preview_first_row").toDouble();
    result["preview_last_row"] = preview->property("preview_last_row").toDouble();
  }
  QFile file(prefix+"panel.json");
  if (file.open(QIODevice::WriteOnly)) file.write(QJsonDocument(result).toJson());
}

void MissionPanel::send(const QString& action) {
  if (!connected_ || !pending_.isEmpty()) return;
  if (commands_->get_subscription_count() == 0) {
    command_error_ = "Command receiver unavailable. Try again when connected.";
    error_->setText(command_error_); return;
  }
  command_error_.clear();
  pending_ = QUuid::createUuid().toString(QUuid::WithoutBraces);
  QJsonObject object{{"id", pending_}, {"action", action}};
  if (action == "start") {
    object["start_m"] = start_->value(); object["distance_m"] = distance_->value();
    object["mode"] = mode_->currentData().toString();
  }
  std_msgs::msg::String msg; msg.data = QJsonDocument(object).toJson(QJsonDocument::Compact).toStdString();
  commands_->publish(msg);
  command_watchdog_->start(); updateControls();
}

void MissionPanel::showStatus(const QString& text) {
  const auto doc = QJsonDocument::fromJson(text.toUtf8()); if (!doc.isObject()) return;
  const auto object = doc.object(); const QString state = object["state"].toString();
  last_state_ = state;
  connected_ = true; watchdog_->start();
  pitch_ = object["scan_pitch_m"].toDouble(); updateExtent();
  const auto task = object["task"].toObject();
  if ((state == "starting" || state == "running" || state == "paused" || state == "draining") && task["mode"].toString() == "wall") {
    const auto target = task["target_x_m"].toArray();
    if (target.size() == 2)
      extent_->setText(QString("Target wall: %1–%2 m, upper 240 degrees\nPlanned vehicle travel: %3–%4 m\nCoverage must be verified from saved measurements.")
        .arg(target[0].toDouble(),0,'f',3).arg(target[1].toDouble(),0,'f',3)
        .arg(task["start_m"].toDouble(),0,'f',3).arg(task["end_m"].toDouble(),0,'f',3));
  }
  if (!pending_.isEmpty() && object["command_result"].toObject()["id"].toString() == pending_) {
    pending_.clear(); command_watchdog_->stop(); command_error_.clear();
  }
  updateControls();
  const QJsonObject labels{{"idle", "Ready"}, {"starting", "Initializing"}, {"running", "Running"},
    {"paused", "Paused"}, {"draining", "Saving pending images"},
    {"complete", "Capture complete; correction before stitching"}, {"stopped", "Stopped early; raw data saved"}, {"failed", "Failed"}};
  status_->setText("Status: "+labels[state].toString(state)+(object["dynamics_only"].toBool() ? " (dynamics only; no images)" : ""));
  const double travelled = object["distance_estimated_m"].toDouble();
  const double distance = state == "idle" ? distance_->value() : object["task"].toObject()["distance_m"].toDouble();
  progress_->setText(QString("Travel: %1 m; remaining: %2 m\nSpeed: %3 m/s; scan: %4 rad/s\nRows: generated %5 / saved %6\nImaging lag: %7 s\nScan angle: %8 rad")
    .arg(travelled,0,'f',3).arg(std::max(0.,distance-travelled),0,'f',3)
    .arg(object["speed_m_s"].toDouble(),0,'f',3).arg(object["scan_rate_rad_s"].toDouble(),0,'f',3)
    .arg(object["rows_generated"].toDouble(),0,'f',0).arg(object["rows_saved"].toDouble(),0,'f',0)
    .arg(object["imaging_lag_s"].toDouble(),0,'f',3).arg(object["scan_rad"].toDouble(),0,'f',3));
  output_->setText("Output: "+object["output"].toString());
  error_->setText(command_error_.isEmpty() ? object["error"].toString() : command_error_);
}

void MissionPanel::updateControls() {
  const bool terminal = last_state_ == "idle" || last_state_ == "complete" || last_state_ == "stopped" || last_state_ == "failed";
  const bool available = connected_ && pending_.isEmpty();
  start_->setEnabled(terminal && available); distance_->setEnabled(terminal && available);
  mode_->setEnabled(terminal && available);
  begin_->setEnabled(terminal && available); pause_->setEnabled(last_state_ == "running" && available);
  resume_->setEnabled(last_state_ == "paused" && available);
  stop_->setEnabled((last_state_ == "running" || last_state_ == "paused") && available);
}

void MissionPanel::updateExtent() {
  const double start = start_->value(), end = start+distance_->value();
  const bool wall = mode_->currentData().toString() == "wall";
  start_label_->setText(wall ? "Wall target start" : "Start position");
  distance_label_->setText(wall ? "Wall target length" : "Travel distance");
  start_->setToolTip(wall ? "Target wall coordinate; vehicle starts earlier in the buffer." : "Vehicle is placed here when you click Start.");
  if (wall) {
    extent_->setText(QString("Target wall: %1–%2 m, upper 240 degrees\nVehicle overscan is planned on Start.\nCoverage must be verified after capture.")
                    .arg(start,0,'f',3).arg(end,0,'f',3));
    return;
  }
  QString coverage = "Coverage: waiting for configuration";
  if (pitch_ > 0) {
    coverage = distance_->value() > 2*pitch_
      ? QString("Conservative full-angle coverage: %1–%2 m").arg(start+pitch_,0,'f',3).arg(end-pitch_,0,'f',3)
      : "No conservative full-angle interval for this short travel";
  }
  extent_->setText(QString("Expected vehicle endpoint: %1 m\n%2\nCoverage must be verified after capture.").arg(end,0,'f',3).arg(coverage));
}
}
PLUGINLIB_EXPORT_CLASS(ssb_rviz::MissionPanel, rviz_common::Panel)
