// Gazebo side of the tunnel scan (DESIGN.md §2, §8.2). Each physics step this system
// commands the synchronised motion profile and pushes the actual joint state into the
// imaging pipeline. Gazebo never waits for imaging; when the server shuts down, the
// destructor waits until every row is on disk.
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include "ssb_core/optix_renderer.hpp"
#include "ssb_core/pipeline.hpp"

namespace ssb_gazebo {

class ScanSystem final : public gz::sim::System,
                         public gz::sim::ISystemConfigure,
                         public gz::sim::ISystemPreUpdate,
                         public gz::sim::ISystemPostUpdate {
 public:
  ~ScanSystem() override {
    if (!pipeline_) return;
    try {
      pipeline_->Finish();
      const auto summary = pipeline_->Wait();
      std::cout << "[ssb] session complete: " << summary.at("rows") << " rows, "
                << summary.at("performance").at("wall_seconds_after_finish") << " s after dynamics ended"
                << std::endl;
    } catch (const std::exception& e) {
      std::cerr << "[ssb] session FAILED: " << e.what() << std::endl;
    }
  }

  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager&) override {
    auto param = [&](const char* env, const char* element) -> std::string {
      if (const char* v = std::getenv(env)) return v;
      return sdf->HasElement(element) ? sdf->Get<std::string>(element) : "";
    };
    const std::string config_path = param("SSB_CONFIG", "config");
    const std::string session = param("SSB_SESSION", "session");
    if (config_path.empty() || session.empty())
      throw std::runtime_error("ssb ScanSystem needs SSB_CONFIG and SSB_SESSION (or <config>/<session>)");
    config_ = ssb::Config::Load(config_path);

    gz::sim::Model model(entity);
    auto joint = [&](const char* element) {
      const auto name = sdf->Get<std::string>(element);
      gz::sim::Joint j(model.JointByName(ecm, name));
      if (!j.Valid(ecm)) throw std::runtime_error("ssb ScanSystem: joint not found: " + name);
      j.EnablePositionCheck(ecm, true);
      j.EnableVelocityCheck(ecm, true);
      return j;
    };
    carriage_ = joint("carriage_joint");
    scan_ = joint("scan_joint");
    wheel_ = joint("wheel_joint");
    scan_.ResetPosition(ecm, {config_.start_theta_rad});

    const double omega = config_.NominalOmega();
    const double commanded_speed = config_.advance_per_rev_m * omega / (2 * M_PI);
    scan_rate_ = omega;
    wheel_rate_ = commanded_speed / (0.5 * config_.calibration.wheel_diameter_m);  // controller uses T_hat
    car_rate_ = wheel_rate_ * 0.5 * config_.truth.wheel_diameter_m;                 // ideal rolling, T_true

    auto renderer = std::make_unique<ssb::OptixRenderer>(config_, ssb::DefaultPtxPath(),
                                                         static_cast<size_t>(config_.batch_rows));
    pipeline_ = std::make_unique<ssb::Pipeline>(config_, std::move(renderer),
                                                ssb::PipelineOptions{session, {"gz", config_path}, "gazebo"});
    std::cout << "[ssb] imaging session " << session << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override {
    if (info.paused) return;
    // gz-sim advances simTime before running systems, so simTime here is already the
    // end of the step being commanded; DART reaches the commanded velocity at that time.
    const double t = std::chrono::duration<double>(info.simTime).count();
    const double end = config_.profile.back()[0];
    const double factor = t <= end ? ssb::EvaluateProfile(config_.profile, t).factor : 0.0;
    scan_.SetVelocity(ecm, {scan_rate_ * factor});
    wheel_.SetVelocity(ecm, {wheel_rate_ * factor});
    carriage_.SetVelocity(ecm, {car_rate_ * factor});
  }

  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override {
    if (info.paused || finished_) return;
    const double t = std::chrono::duration<double>(info.simTime).count();
    const auto x = carriage_.Position(ecm), v = carriage_.Velocity(ecm);
    const auto th = scan_.Position(ecm), w = scan_.Velocity(ecm);
    const auto wp = wheel_.Position(ecm), wv = wheel_.Velocity(ecm);
    if (!x || !v || !th || !w || !wp || !wv || x->empty() || th->empty() || wp->empty()) return;
    pipeline_->Push({t, config_.start_x_m + x->front(), v->front(), th->front(), w->front(), wp->front(),
                     wv->front()});
    if (t >= config_.profile.back()[0]) {
      finished_ = true;
      pipeline_->Finish();
      std::cout << "[ssb] motion profile complete at sim time " << t << " s; imaging continues" << std::endl;
    }
  }

 private:
  ssb::Config config_;
  std::unique_ptr<ssb::Pipeline> pipeline_;
  gz::sim::Joint carriage_, scan_, wheel_;
  double scan_rate_ = 0, wheel_rate_ = 0, car_rate_ = 0;
  bool finished_ = false;
};

}  // namespace ssb_gazebo

GZ_ADD_PLUGIN(ssb_gazebo::ScanSystem, gz::sim::System, ssb_gazebo::ScanSystem::ISystemConfigure,
              ssb_gazebo::ScanSystem::ISystemPreUpdate, ssb_gazebo::ScanSystem::ISystemPostUpdate)
