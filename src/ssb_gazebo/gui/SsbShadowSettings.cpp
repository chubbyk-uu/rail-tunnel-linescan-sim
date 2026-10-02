#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/Plugin.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Image.hh>
#include <gz/rendering/Scene.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <chrono>
#include <cstdlib>
#include <sstream>
#include "shadow_settings.hpp"

namespace ssb {
class SsbShadowSettings : public gz::gui::Plugin {
 public:
  void LoadConfig(const tinyxml2::XMLElement* config) override {
    if (auto* value = config->FirstChildElement("apply_fix")) value->QueryBoolText(&apply_fix_);
    if (auto* value = config->FirstChildElement("diagnostic")) value->QueryBoolText(&diagnostic_);
    if (diagnostic_) publisher_ = node_.Advertise<gz::msgs::Image>("/strip_check/image");
    // Opt-in measurement only: no extra rendering, image readback or file I/O.
    if(const char* topic=std::getenv("SSB_GUI_FRAME_TOPIC"))
      frame_publisher_=node_.Advertise<gz::msgs::StringMsg>(topic);
    if (auto* window = gz::gui::App()->findChild<gz::gui::MainWindow*>())
      window->installEventFilter(this);
  }
 protected:
  bool eventFilter(QObject* object, QEvent* event) override {
    if (event->type() == gz::gui::events::Render::kType) {
      if (apply_fix_ && ApplyStripShadowSettings())
        gzmsg << "SsbShadowSettings: narrow-beam normal offset = 1 (GUI only)" << std::endl;
      if (diagnostic_) PublishImage();
      if(frame_publisher_ && gz::rendering::sceneFromFirstRenderEngine()) PublishFrameRate();
    }
    return QObject::eventFilter(object, event);
  }
 private:
  void PublishFrameRate() {
    const auto now=std::chrono::steady_clock::now();
    if(frame_begin_==std::chrono::steady_clock::time_point{}) {frame_begin_=now;return;}
    ++frames_;
    const double seconds=std::chrono::duration<double>(now-frame_begin_).count();
    if(seconds<1.) return;
    std::ostringstream data;
    data<<"{\"frames\":"<<frames_<<",\"seconds\":"<<seconds<<",\"fps\":"<<frames_/seconds<<"}";
    gz::msgs::StringMsg message;message.set_data(data.str());
    frame_publisher_.Publish(message);frame_begin_=now;frames_=0;
  }
  void PublishImage() {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_image_ < std::chrono::milliseconds(100)) return;
    if (!camera_) {
      auto scene = gz::rendering::sceneFromFirstRenderEngine();
      if (!scene) return;
      for (unsigned i = 0; i < scene->NodeCount(); ++i) {
        auto candidate = std::dynamic_pointer_cast<gz::rendering::Camera>(scene->NodeByIndex(i));
        if (candidate && candidate->HasUserData("user-camera")) {
          camera_ = candidate;
          camera_->SetHFOV(1.05);
          return; // First copy must follow a render with the fixed FOV.
        }
      }
    }
    if (!camera_) return;
    auto image = camera_->CreateImage();
    camera_->Copy(image); // Read the completed frame, without rendering again.
    gz::msgs::Image message;
    message.set_width(image.Width()); message.set_height(image.Height());
    message.set_step(image.Width()*3); message.set_pixel_format_type(gz::msgs::RGB_INT8);
    message.set_data(image.Data(), image.MemorySize());
    publisher_.Publish(message); last_image_ = now;
  }
  bool apply_fix_ = true, diagnostic_ = false;
  gz::transport::Node node_;
  gz::transport::Node::Publisher publisher_;
  gz::transport::Node::Publisher frame_publisher_;
  uint64_t frames_=0;
  std::chrono::steady_clock::time_point frame_begin_{};
  gz::rendering::CameraPtr camera_;
  std::chrono::steady_clock::time_point last_image_{};
};
}  // namespace ssb

GZ_ADD_PLUGIN(ssb::SsbShadowSettings, gz::gui::Plugin)
