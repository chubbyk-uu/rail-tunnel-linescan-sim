#include "ssb_core/kinematic.hpp"

#include <cmath>

namespace ssb {

KinematicSource::KinematicSource(const Config& config) : config_(config) {
  config_.Validate();
  omega_ = config_.NominalOmega();
  const double commanded_speed = config_.advance_per_rev_m * omega_ / (2 * M_PI);
  wheel_omega_ = commanded_speed / (0.5 * config_.calibration.wheel_diameter_m);
  car_speed_ = wheel_omega_ * 0.5 * config_.truth.wheel_diameter_m;
  end_time_ = config_.profile.back()[0];
}

PoseSample KinematicSource::At(double t) const {
  const ProfileState p = EvaluateProfile(config_.profile, t);
  PoseSample s;
  s.t = t;
  s.theta = config_.start_theta_rad + omega_ * p.integral;
  s.omega = omega_ * p.factor;
  s.x = config_.start_x_m + car_speed_ * p.integral;
  s.v = car_speed_ * p.factor;
  s.wheel = wheel_omega_ * p.integral;
  s.wheel_omega = wheel_omega_ * p.factor;
  return s;
}

std::vector<PoseSample> KinematicSource::Sample() const {
  std::vector<PoseSample> out;
  const auto n = static_cast<long long>(std::floor(end_time_ / config_.sample_period_s + 1e-9));
  out.reserve(n + 1);
  for (long long i = 0; i <= n; ++i) out.push_back(At(i * config_.sample_period_s));
  return out;
}

}  // namespace ssb
