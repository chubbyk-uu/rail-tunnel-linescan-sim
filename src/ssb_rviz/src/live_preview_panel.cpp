#include "live_preview_panel.hpp"
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <QDockWidget>
#include <QMainWindow>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QPixmap>
#include <QResizeEvent>
#include <QVBoxLayout>

namespace ssb_rviz {
LivePreviewPanel::LivePreviewPanel(QWidget* parent) : Panel(parent) {
  auto* layout = new QVBoxLayout(this);
  auto* note = new QLabel("Raw Mono8 preview: latest saved block.\nReduced resolution; no optical correction.");
  note->setWordWrap(true); layout->addWidget(note);
  image_label_ = new QLabel("Waiting for capture");
  image_label_->setObjectName("raw_preview_image");
  image_label_->setAlignment(Qt::AlignCenter);
  image_label_->setMinimumSize(256, 200);
  image_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  image_label_->setStyleSheet("background: #181818; color: #b8b8b8;");
  layout->addWidget(image_label_, 1);
  caption_ = new QLabel("No active capture"); caption_->setWordWrap(true);
  caption_->setObjectName("raw_preview_caption"); layout->addWidget(caption_);
  setMinimumWidth(280);
  connect(this, &LivePreviewPanel::received, this, &LivePreviewPanel::showPreview, Qt::QueuedConnection);
  connect(this, &LivePreviewPanel::missionReceived, this, &LivePreviewPanel::showMission, Qt::QueuedConnection);
  watchdog_ = new QTimer(this); watchdog_->setSingleShot(true); watchdog_->setInterval(3500);
  connect(watchdog_, &QTimer::timeout, this, [this] {
    image_ = QImage(); paintImage(); caption_->setText("Preview disconnected");
  });
}

void LivePreviewPanel::onInitialize() {
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  preview_ = node->create_subscription<std_msgs::msg::String>("/ssb/mission/preview",
    rclcpp::QoS(1).best_effort(), [this](std_msgs::msg::String::ConstSharedPtr msg) {
      Q_EMIT received(QString::fromStdString(msg->data));
    });
  status_ = node->create_subscription<std_msgs::msg::String>("/ssb/mission/status",
    rclcpp::QoS(1).transient_local(), [this](std_msgs::msg::String::ConstSharedPtr msg) {
      Q_EMIT missionReceived(QString::fromStdString(msg->data));
    });
  // Keep the compact raw thumbnail below Displays; mission controls occupy the right.
  QTimer::singleShot(0, this, [this] {
    QWidget* parent = parentWidget();
    while (parent && !qobject_cast<QDockWidget*>(parent)) parent = parent->parentWidget();
    auto* main = qobject_cast<QMainWindow*>(window());
    if (!main || !parent) return;
    auto* preview = qobject_cast<QDockWidget*>(parent);
    main->addDockWidget(Qt::LeftDockWidgetArea, preview);
    for (auto* dock : main->findChildren<QDockWidget*>()) {
      if (dock->windowTitle() == "Displays" && main->dockWidgetArea(dock) == Qt::LeftDockWidgetArea) {
        main->splitDockWidget(dock, preview, Qt::Vertical);
        main->resizeDocks({dock, preview}, {350, 370}, Qt::Vertical);
        break;
      }
    }
  });
}

void LivePreviewPanel::showMission(const QString& text) {
  const auto data = QJsonDocument::fromJson(text.toUtf8()).object();
  if (!data.contains("output")) return;
  const auto output = data["output"].toString();
  if (output == output_) return;
  output_ = output; image_ = QImage(); paintImage();
  image_label_->setProperty("preview_output", output_);
  caption_->setText("Waiting for first saved raw block");
}

void LivePreviewPanel::showPreview(const QString& text) {
  const auto data = QJsonDocument::fromJson(text.toUtf8()).object();
  if (data["output"].toString() != output_) return; // discard a previous task's delayed image
  watchdog_->start();
  if (data["status"].toString() != "ready") {
    image_ = QImage(); paintImage();
    caption_->setText(data["status"].toString() == "disabled" ? "No images in dynamics-only mode" :
      data["status"].toString() == "error" ? "Preview error: "+data["error"].toString() : "Waiting for saved raw block");
    return;
  }
  const auto encoded = data["png"].toString();
  if (data["encoding"].toString() != "png;base64" || encoded.size() > 512*1024) return;
  auto image = QImage::fromData(QByteArray::fromBase64(encoded.toLatin1()), "PNG");
  if (image.isNull() || image.width() > 512 || image.height() > 512) return;
  image_ = image;
  const auto size = data["source_size"].toArray();
  caption_->setText(QString("Rows %1–%2\nSource: %3 × %4 px\nPreview: %5 × %6 px · 1 Hz max")
    .arg(data["first_row"].toDouble(),0,'f',0).arg(data["last_row"].toDouble(),0,'f',0)
    .arg(size[0].toInt()).arg(size[1].toInt()).arg(image.width()).arg(image.height()));
  image_label_->setProperty("preview_first_row", data["first_row"].toDouble());
  image_label_->setProperty("preview_last_row", data["last_row"].toDouble());
  paintImage();
}

void LivePreviewPanel::paintImage() {
  image_label_->setProperty("preview_ready", !image_.isNull());
  if (image_.isNull()) { image_label_->clear(); image_label_->setText("Waiting for capture"); return; }
  image_label_->setPixmap(QPixmap::fromImage(image_).scaled(image_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

void LivePreviewPanel::resizeEvent(QResizeEvent* event) {
  Panel::resizeEvent(event); paintImage();
}
}
PLUGINLIB_EXPORT_CLASS(ssb_rviz::LivePreviewPanel, rviz_common::Panel)
