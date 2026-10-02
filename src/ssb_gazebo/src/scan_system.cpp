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
#include <future>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "ssb_core/optix_renderer.hpp"
#include "ssb_core/pipeline.hpp"
#include "assembly_check.hpp"
#include "control_math.hpp"

namespace ssb_gazebo {

class ScanSystem final : public gz::sim::System,
                         public gz::sim::ISystemConfigure,
                         public gz::sim::ISystemPreUpdate,
                         public gz::sim::ISystemPostUpdate {
 public:
  ~ScanSystem() override {
    if (!pipeline_) return;
    try {
      if (!completion_.valid()) FinishInBackground();
      const auto summary = completion_.get();
      std::cout << "[ssb] session data complete, motion complete: " << summary.at("motion").at("complete") << ", "
                << summary.at("rows") << " rows, "
                << summary.at("performance").at("wall_seconds_after_finish") << " s after dynamics ended"
                << std::endl;
    } catch (const std::exception& e) {
      std::cerr << "[ssb] session FAILED: " << e.what() << std::endl;
    }
  }

  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& events) override {
    // A throw here aborts the server (and on WSL leaves a crash dump); refuse cleanly.
    try {
      ConfigureOrThrow(entity, sdf, ecm, events);
    } catch (const std::exception& e) {
      std::cerr << "[ssb] refusing to run: " << e.what() << std::endl;
      std::_Exit(2);
    }
  }

 private:
  void ConfigureOrThrow(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                        gz::sim::EntityComponentManager& ecm, gz::sim::EventManager&) {
    auto param = [&](const char* env, const char* element) -> std::string {
      if (const char* v = std::getenv(env)) return v;
      return sdf->HasElement(element) ? sdf->Get<std::string>(element) : "";
    };
    const std::string config_path = param("SSB_CONFIG", "config");
    const std::string session = param("SSB_SESSION", "session");
    if (config_path.empty() || session.empty())
      throw std::runtime_error("ssb ScanSystem needs SSB_CONFIG and SSB_SESSION (or <config>/<session>)");
    config_ = ssb::Config::Load(config_path);
    step_ = config_.sample_period_s;

    gz::sim::Model model(entity);
    CheckAssembly(model,ecm,config_);
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
    if (sdf->HasElement("follower_wheel_joint")) {
      for (auto element = sdf->FindElement("follower_wheel_joint"); element;
           element = element->GetNextElement("follower_wheel_joint")) {
        gz::sim::Joint follower(model.JointByName(ecm, element->Get<std::string>()));
        if (!follower.Valid(ecm)) throw std::runtime_error("ssb ScanSystem: follower wheel joint not found");
        follower_wheels_.push_back(follower);
      }
    }
    scan_.ResetPosition(ecm, {config_.start_theta_rad});

    const double omega = config_.NominalOmega();
    const double commanded_speed = config_.advance_per_rev_m * omega / (2 * M_PI);
    scan_rate_ = omega;
    wheel_rate_ = commanded_speed / (0.5 * config_.calibration.wheel_diameter_m);  // controller uses T_hat
    car_rate_ = wheel_rate_ * 0.5 * config_.truth.wheel_diameter_m;                 // ideal rolling, T_true

    config_path_ = config_path;
    session_ = session;
  }

  // One step source: the pose stream is sampled at every physics step, so the actual
  // step must equal motion.sample_period_s, or the run silently covers the wrong span.
  // Checked on the first step against info.dt (the world's Physics component does not
  // exist yet at Configure); the pipeline, and so the session, is only created after.
  void StartOrExit(const gz::sim::UpdateInfo& info) {
    try {
      const double dt = std::chrono::duration<double>(info.dt).count();
      if (std::abs(dt - step_) > 1e-12)
        throw std::runtime_error("physics step " + std::to_string(dt) + " s differs from motion.sample_period_s " +
                                 std::to_string(step_) + " s");
      auto renderer = std::make_unique<ssb::OptixRenderer>(config_, ssb::DefaultPtxPath(),
                                                           static_cast<size_t>(config_.batch_rows));
      ssb::PipelineOptions options{session_, {"gz", config_path_}, "gazebo"};
      options.inputs["config"] = ssb::FileIdentity(config_path_);
      if (const char* world = std::getenv("SSB_WORLD")) options.inputs["world"] = ssb::FileIdentity(world);
      options.planned_end_s = config_.profile.back()[0];
      pipeline_ = std::make_unique<ssb::Pipeline>(config_, std::move(renderer), options);
      std::cout << "[ssb] imaging session " << session_ << std::endl;
    } catch (const std::exception& e) {
      std::cerr << "[ssb] refusing to run: " << e.what() << std::endl;
      std::_Exit(2);
    }
  }

  // Complete and measure capture when motion ends, without closing the GUI or
  // blocking physics on queued rendering. Destructor joins before pipeline dies.
  void FinishInBackground(const std::string& error = "") {
    if (completion_.valid()) return;
    pipeline_->Finish(error);
    completion_ = std::async(std::launch::async, [this] { return pipeline_->Wait(); });
  }

 public:
  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override {
    if (info.paused) return;
    if (finished_) {
      scan_.SetVelocity(ecm, {0});
      wheel_.SetVelocity(ecm, {0});
      carriage_.SetVelocity(ecm, {0});
      for (auto& follower : follower_wheels_) follower.SetVelocity(ecm, {0});
      return;
    }
    if (!pipeline_) StartOrExit(info);
    // gz-sim advances simTime before running systems, so simTime here is already the
    // end of the step being commanded; DART reaches the commanded velocity at that time.
    const double t = std::chrono::duration<double>(info.simTime).count();
    const double end = config_.profile.back()[0];
    const double factor = t <= end ? ssb::EvaluateProfile(config_.profile, t).factor : 0.0;
    scan_.SetVelocity(ecm, {scan_rate_ * factor});
    wheel_.SetVelocity(ecm, {wheel_rate_ * factor});
    for (auto& follower : follower_wheels_) follower.SetVelocity(ecm, {wheel_rate_ * factor});
    carriage_.SetVelocity(ecm, {car_rate_ * factor});
  }

  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override {
    if (info.paused || finished_) return;
    const double t = std::chrono::duration<double>(info.simTime).count();
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (std::abs(dt - step_) > 1e-12) {
      // Step changed at run time: stop feeding; the session reports the motion as incomplete.
      std::cerr << "[ssb] physics step changed to " << dt << " s at sim time " << t << "; capture stopped"
                << std::endl;
      finished_ = true;
      FinishInBackground("physics step changed during acquisition");
      return;
    }
    const auto x = carriage_.Position(ecm), v = carriage_.Velocity(ecm);
    const auto th = scan_.Position(ecm), w = scan_.Velocity(ecm);
    const auto wp = wheel_.Position(ecm), wv = wheel_.Velocity(ecm);
    if (!HasValues(x, v, th, w, wp, wv)) {
      finished_ = true;
      FinishInBackground("joint position or velocity unavailable during acquisition");
      return;
    }
    pipeline_->Push({t, config_.start_x_m + x->front(), v->front(), th->front(), w->front(), wp->front(),
                     wv->front()});
    if (t >= config_.profile.back()[0]) {
      finished_ = true;
      FinishInBackground();
      std::cout << "[ssb] motion profile complete at sim time " << t << " s; imaging continues" << std::endl;
    }
  }

 private:
  ssb::Config config_;
  std::string config_path_, session_;
  std::unique_ptr<ssb::Pipeline> pipeline_;
  std::future<nlohmann::json> completion_;
  gz::sim::Joint carriage_, scan_, wheel_;
  std::vector<gz::sim::Joint> follower_wheels_;
  double step_ = 0, scan_rate_ = 0, wheel_rate_ = 0, car_rate_ = 0;
  bool finished_ = false;
};

}  // namespace ssb_gazebo

GZ_ADD_PLUGIN(ssb_gazebo::ScanSystem, gz::sim::System, ssb_gazebo::ScanSystem::ISystemConfigure,
              ssb_gazebo::ScanSystem::ISystemPreUpdate, ssb_gazebo::ScanSystem::ISystemPostUpdate)
