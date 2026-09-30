#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <variant>

#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/Plugin.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/LensFlarePass.hh>
#include <gz/rendering/Light.hh>
#include <gz/rendering/RayQuery.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderPassSystem.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>

namespace ssb {
// Display only: one GPU post-process for the strongest visible emitter. It does
// not attach to sensors, change scene illumination, or touch OptiX acquisition.
class SsbLightGlare : public gz::gui::Plugin {
 Q_OBJECT
 Q_PROPERTY(bool glareEnabled READ GlareEnabled WRITE SetGlareEnabled NOTIFY GlareEnabledChanged)
 public: bool GlareEnabled() const { return enabled_.load(); }
 public: Q_INVOKABLE void SetGlareEnabled(bool value) {
   if (enabled_.exchange(value) != value) {
     gzmsg << "SsbLightGlare: " << (value ? "enabled" : "disabled") << "\n";
     emit GlareEnabledChanged();
   }
 }
 signals: void GlareEnabledChanged();
 public: void LoadConfig(const tinyxml2::XMLElement *) override {
   if (auto *window = gz::gui::App()->findChild<gz::gui::MainWindow *>())
     window->installEventFilter(this);
 }

 protected: bool eventFilter(QObject *object, QEvent *event) override {
   // MinimalScene emits Render after completing camera.Update(). All rendering
   // APIs below run on that thread, and changes apply to the next frame.
   if (event->type() == gz::gui::events::Render::kType) Update();
   return QObject::eventFilter(object, event);
 }

 private: bool Initialize() {
   scene_ = gz::rendering::sceneFromFirstRenderEngine();
   if (!scene_) return false;
   for (unsigned i = 0; i < scene_->NodeCount(); ++i) {
     auto candidate = std::dynamic_pointer_cast<gz::rendering::Camera>(scene_->NodeByIndex(i));
     if (candidate && candidate->HasUserData("user-camera")) {
       const auto data = candidate->UserData("user-camera");
       if (const auto *flag = std::get_if<bool>(&data); flag && *flag) {
         camera_ = candidate;
         break;
       }
     }
   }
   if (!camera_) return false;
   auto system = scene_->Engine()->RenderPassSystem();
   if (!system) return false;
   pass_ = std::dynamic_pointer_cast<gz::rendering::LensFlarePass>(
       system->Create<gz::rendering::LensFlarePass>());
   if (!pass_) return false;
   pass_->Init(scene_);
   pass_->SetEnabled(false);
   pass_->SetColor({1.0, .96, .90});
   // Native Ogre2 occlusion compares distances from the world origin. Use a
   // camera-relative segment test instead, so translating the robot is safe.
   pass_->SetOcclusionSteps(0);
   marker_ = scene_->CreatePointLight("ssb_gui_glare_marker");
   marker_->SetIntensity(0);
   marker_->SetCastShadows(false);
   marker_->SetAttenuationRange(.001);
   scene_->RootVisual()->AddChild(marker_);
   pass_->SetLight(marker_);
   query_ = scene_->CreateRayQuery();
   query_->SetPreferGpu(true);
   camera_->AddRenderPass(pass_);
   gzmsg << "SsbLightGlare: GUI-only lens flare attached to user camera\n";
   return true;
 }

 private: void Update() {
   if (!enabled_.load()) {
     if (pass_) pass_->SetEnabled(false);
     return;
   }
   if (!pass_ && !Initialize()) return;
   const auto now = std::chrono::steady_clock::now();
   if (now - lastVisibility_ >= std::chrono::milliseconds(200)) {
     lastVisibility_ = now;
     for (auto &source : sources_) {
       if (!source.visual) {
         source.visual = scene_->VisualByName(source.name);
         if (source.visual) gzmsg << "SsbLightGlare: emitter " << source.name << " found\n";
       }
       source.visible = false;
       if (!source.visual) continue;
       gz::math::Vector3d point, direction;
       double distance, facing;
       if (!Candidate(source, point, direction, distance, facing)) continue;
       const auto local = camera_->WorldRotation().Inverse() * direction;
       const double tangent = std::tan(camera_->HFOV().Radian() / 2);
       query_->SetFromCamera(camera_, {-local.Y() / (local.X() * tangent),
           local.Z() * camera_->AspectRatio() / (local.X() * tangent)});
       const auto hit = query_->ClosestPoint(false);
       source.visible = hit.distance < 0 ||
           (hit.point - camera_->WorldPosition()).Length() >= distance - .002;
     }
   }
   double best = 0;
   gz::math::Vector3d bestPoint;
   for (const auto &source : sources_) {
     if (!source.visible) continue;
     gz::math::Vector3d point, direction;
     double distance, facing;
     if (!Candidate(source, point, direction, distance, facing)) continue;
     const double score = std::pow(facing, 4) / std::max(.25, distance * distance);
     if (score > best) { best = score; bestPoint = point; }
   }
   pass_->SetEnabled(best > 0);
   if (best > 0) {
     marker_->SetWorldPosition(bestPoint);
     pass_->SetScale(std::clamp(.15 * std::sqrt(best), .008, .20));
     if (!reportedVisible_) {
       gzmsg << "SsbLightGlare: visible emitter, scale=" << pass_->Scale() << "\n";
       reportedVisible_ = true;
     }
   }
 }

 private: struct Source {
   const char *name;
   double frontOffset;
   gz::rendering::VisualPtr visual;
   bool visible = false;
 };

 private: bool Candidate(const Source &source, gz::math::Vector3d &point,
     gz::math::Vector3d &direction, double &distance, double &facing) const {
   if (!source.visual) return false;
   const auto normal = source.visual->WorldRotation() * gz::math::Vector3d::UnitZ;
   point = source.visual->WorldPosition() + normal * source.frontOffset;
   direction = point - camera_->WorldPosition();
   distance = direction.Length();
   if (distance < .02) return false;
   direction /= distance;
   facing = -normal.Dot(direction);
   if (facing <= .2) return false;
   const auto local = camera_->WorldRotation().Inverse() * direction;
   if (local.X() <= 0) return false;
   const double tangent = std::tan(camera_->HFOV().Radian() / 2);
   return std::abs(local.Y() / local.X()) < tangent &&
       std::abs(local.Z() / local.X()) < tangent / camera_->AspectRatio();
 }

 private: gz::rendering::ScenePtr scene_;
 private: gz::rendering::CameraPtr camera_;
 private: gz::rendering::LensFlarePassPtr pass_;
 private: gz::rendering::PointLightPtr marker_;
 private: gz::rendering::RayQueryPtr query_;
 private: std::chrono::steady_clock::time_point lastVisibility_{};
 private: bool reportedVisible_ = false;
 private: std::atomic<bool> enabled_{false};
 private: std::array<Source, 5> sources_{{
   {"scan_car::base::work_-1_-1_glass", .004, nullptr},
   {"scan_car::base::work_-1_1_glass", .004, nullptr},
   {"scan_car::base::work_1_-1_glass", .004, nullptr},
   {"scan_car::base::work_1_1_glass", .004, nullptr},
   {"scan_car::head::lamp_window", .020, nullptr}}};
};
}  // namespace ssb
GZ_ADD_PLUGIN(ssb::SsbLightGlare, gz::gui::Plugin)
#include "SsbLightGlare.moc"
