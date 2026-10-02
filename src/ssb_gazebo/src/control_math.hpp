#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace ssb_gazebo {

template<class... Values>
bool HasValues(const Values&... values) {
  return (... && (values && !values->empty()));
}


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
