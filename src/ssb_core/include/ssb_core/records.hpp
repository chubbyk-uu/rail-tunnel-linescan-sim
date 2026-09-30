#pragma once
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

// Fixed binary records written by a session. Every file's numpy dtype is written
// next to it from the Fields() lists below, so readers never hardcode a layout.
namespace ssb {

// One pose-stream sample (the true motion Gazebo or a kinematic source produced).
struct PoseSample {
  double t = 0;
  double x = 0, v = 0;               // car reference along the track and its rate
  double theta = 0, omega = 0;       // scan axis angle (unwrapped) and rate
  double wheel = 0, wheel_omega = 0; // left/rear odometer wheel angle and rate
  // v2 body pose: world base-link position/orientation; ZYX Euler rates.
  double y=0,z=0,roll=0,pitch=0,yaw=0,vy=0,vz=0,roll_rate=0,pitch_rate=0,yaw_rate=0;
  double body_valid=0,right_wheel=0,right_wheel_omega=0;
};

struct EdgeRecord {
  double t;
  int64_t count;  // relative count after the edge (0 at power-on)
  int64_t dir;    // +1 / -1
};

enum GateKind : int32_t { kGateStart = 0, kGateEnd = 1 };
struct GateRecord {
  double t;
  int64_t revolution;
  int32_t kind;
  int32_t dir;
};

enum DropReason : int32_t {
  kDropEarlyEdge = 1,  // next encoder edge arrived before this interpolated sub-tick
  kDropNoPeriod = 2,   // no valid period estimate (start-up or edge gap > max_period)
  kDropReverse = 3,    // scan axis reversed
  kDropOverrun = 4,    // trigger faster than the camera's line-rate limit
  kDropStreamEnd = 5,  // pose stream ended before the row could be produced
};
// A lattice row that never became an exposure. [t_lo, t_hi] brackets where it
// would have been; gated says whether the gate was open at either end.
struct DroppedRowRecord {
  int64_t row;
  double t_lo;
  double t_hi;
  int32_t reason;
  int32_t gated;
};

// A recorded exposure. `row` is the rescaler lattice index (sub-tick / divide).
struct RowRecord {
  int64_t sequence;
  int64_t row;
  int64_t segment;  // gate revolution index
  double t_trigger;
  double t_center;
};

// True pose at the exposure centre (evaluation only).
struct RowTruthRecord {
  int64_t sequence;
  double t_center;
  double theta, omega, x, v;
  double y=0,z=0,roll=0,pitch=0,yaw=0,body_valid=0;
};

using Fields = std::vector<std::pair<std::string, std::string>>;  // name, numpy type
inline Fields PoseSampleFields() {
  return {{"t", "<f8"}, {"x", "<f8"}, {"v", "<f8"}, {"theta", "<f8"},
          {"omega", "<f8"}, {"wheel", "<f8"}, {"wheel_omega", "<f8"},
          {"y","<f8"},{"z","<f8"},{"roll","<f8"},{"pitch","<f8"},{"yaw","<f8"},
          {"vy","<f8"},{"vz","<f8"},{"roll_rate","<f8"},{"pitch_rate","<f8"},{"yaw_rate","<f8"},
          {"body_valid","<f8"},{"right_wheel","<f8"},{"right_wheel_omega","<f8"}};
}
inline Fields EdgeFields() { return {{"t", "<f8"}, {"count", "<i8"}, {"dir", "<i8"}}; }
inline Fields GateFields() { return {{"t", "<f8"}, {"revolution", "<i8"}, {"kind", "<i4"}, {"dir", "<i4"}}; }
inline Fields DroppedRowFields() {
  return {{"row", "<i8"}, {"t_lo", "<f8"}, {"t_hi", "<f8"}, {"reason", "<i4"}, {"gated", "<i4"}};
}
inline Fields RowFields() {
  return {{"sequence", "<i8"}, {"row", "<i8"}, {"segment", "<i8"}, {"t_trigger", "<f8"}, {"t_center", "<f8"}};
}
inline Fields RowTruthFields() {
  return {{"sequence", "<i8"}, {"t_center", "<f8"}, {"theta", "<f8"},
          {"omega", "<f8"}, {"x", "<f8"}, {"v", "<f8"},
          {"y","<f8"},{"z","<f8"},{"roll","<f8"},{"pitch","<f8"},{"yaw","<f8"},{"body_valid","<f8"}};
}

static_assert(sizeof(PoseSample) == 160);
static_assert(sizeof(EdgeRecord) == 24);
static_assert(sizeof(GateRecord) == 24);
static_assert(sizeof(DroppedRowRecord) == 32);
static_assert(sizeof(RowRecord) == 40);
static_assert(sizeof(RowTruthRecord) == 96);

}  // namespace ssb
