#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include "ssb_core/config.hpp"

namespace ssb_gazebo {

template<class... Values>
bool HasValues(const Values&... values) {
  return (... && (values && !values->empty()));
}

inline double DriveTorque(double target_rate, double measured_rate) {
  return std::clamp(12*(target_rate-measured_rate), -8., 8.);
}

// Encoder-only PI holds against the small grade force of an irregular rail.
// Conditional integration prevents winding up while the 8 N m limit is active.
class DriveController {
 public:
  double Update(double target_rate,double measured_rate,double dt) {
    const double error=target_rate-measured_rate;
    const double trial=integral_+30*error*dt;
    const double effort=12*error+trial;
    if(std::abs(effort)<=8. || effort*error<0.) integral_=std::clamp(trial,-8.,8.);
    return std::clamp(12*error+integral_,-8.,8.);
  }
 private:
  double integral_=0.;
};

// Only encoder distance/rates and simulation elapsed time enter this controller.
// Timeout is a fault guard; elapsed time can never declare successful completion.
class DistanceController {
 public:
  double Update(const ssb::Config::DistanceStop& stop,double cruise,double elapsed,
                double distance,double speed,double dt,bool scan_settled) {
    const double remaining=stop.target_m-distance;
    const double u=std::clamp(elapsed/stop.ramp_s,0.,1.);
    const double startup=u*u*u*(10+u*(-15+6*u));
    double desired=0.;
    if(remaining>stop.tolerance_m) {
      const double braking_remaining=std::max(0.,remaining-.02*std::max(0.,speed));
      desired=std::min(cruise*startup,cruise*std::sqrt(braking_remaining/stop.brake_distance_m));
      // Resolve the last few encoder increments without an abrupt crossing.
      desired=std::min(desired,remaining/.05);
    }
    command_=desired;
    const bool parked=std::abs(remaining)<=stop.tolerance_m &&
                      std::abs(speed)<=stop.speed_tolerance_m_s && scan_settled;
    hold_=parked ? hold_+dt : 0.;
    complete_=hold_>=stop.hold_s;
    return command_;
  }
  bool Complete() const { return complete_; }
 private:
  double command_=0.,hold_=0.;
  bool complete_=false;
};


inline std::array<double,3> EulerRates(double pitch,double yaw,const std::array<double,3>& world_omega) {
  const double horizontal=std::cos(yaw)*world_omega[0]+std::sin(yaw)*world_omega[1];
  return {horizontal/std::cos(pitch),
          -std::sin(yaw)*world_omega[0]+std::cos(yaw)*world_omega[1],
          world_omega[2]+std::tan(pitch)*horizontal};
}

class ScanServo {
 public:
  double Update(double distance,double angle,double dt,double start,double advance) {
    speed_+=(distance-distance_-speed_*dt)/(.025+dt);
    distance_=distance;
    target_=start+2*M_PI*distance/advance;
    const double rate=2*M_PI*speed_/advance+12*(target_-angle);
    command_+=(std::clamp(rate,0.,3.3)-command_)*dt/(.005+dt);
    return command_;
  }
  double Target() const { return target_; }
 private:
  double distance_=0,speed_=0,target_=0,command_=0;
};
}  // namespace ssb_gazebo
