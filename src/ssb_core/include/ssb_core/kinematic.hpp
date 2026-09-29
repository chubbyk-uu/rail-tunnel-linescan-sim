#pragma once
#include <vector>

#include "ssb_core/config.hpp"
#include "ssb_core/records.hpp"

namespace ssb {

// Ideal synchronised motion from the config profile (no dynamics). The controller
// commands the wheel from the calibrated diameter; the car moves by the true one,
// so a diameter mismatch changes the advance per revolution as on the real robot.
class KinematicSource {
 public:
  explicit KinematicSource(const Config& config);
  PoseSample At(double t) const;
  double EndTime() const { return end_time_; }
  // Samples at t = i * sample_period_s, i = 0 .. floor(EndTime / sample_period_s).
  std::vector<PoseSample> Sample() const;

 private:
  Config config_;
  double omega_, wheel_omega_, car_speed_, end_time_;
};

}  // namespace ssb
