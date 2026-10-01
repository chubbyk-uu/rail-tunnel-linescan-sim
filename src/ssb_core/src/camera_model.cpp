#include "ssb_core/camera_model.hpp"

#include <cmath>

namespace ssb {
namespace {

using Mat3 = std::array<Vec3, 3>;  // rows

Vec3 Mul(const Mat3& m, const Vec3& v) {
  return {m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2], m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
          m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]};
}

Mat3 Mul(const Mat3& a, const Mat3& b) {
  Mat3 r{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) r[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
  return r;
}

Mat3 Rx(double a) {
  const double c = std::cos(a), s = std::sin(a);
  return {{{1, 0, 0}, {0, c, -s}, {0, s, c}}};
}
Mat3 Ry(double a) {
  const double c = std::cos(a), s = std::sin(a);
  return {{{c, 0, s}, {0, 1, 0}, {-s, 0, c}}};
}
Mat3 Rz(double a) {
  const double c = std::cos(a), s = std::sin(a);
  return {{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}};
}

}  // namespace

HeadPose TrueHeadPose(const Config& config, const PoseSample& pose) {
  const auto& m = config.truth.mount;
  const Mat3 tilt = Mul(Rz(m.tilt_z_rad), Ry(m.tilt_y_rad));
  const Mat3 body=pose.body_valid ? Mul(Rz(pose.yaw),Mul(Ry(pose.pitch),Rx(pose.roll))) : Rx(0);
  const Mat3 head = Mul(body,Mul(tilt, Rx(-pose.theta)));
  Vec3 axis_point{pose.x + config.truth.head_mount_x_m, m.dy_m,
                  config.base_reference_z_m + config.scan_axis_height_m + m.dz_m};
  if (pose.body_valid) {
    const auto local=Mul(body,Vec3{config.truth.head_mount_x_m,m.dy_m,config.scan_axis_height_m+m.dz_m});
    axis_point={pose.x+local[0],pose.y+local[1],pose.z+local[2]};
  }
  const Vec3 offset = Mul(head, Vec3{0, m.tangential_m, m.e_m});
  HeadPose h;
  h.origin = {axis_point[0] + offset[0], axis_point[1] + offset[1], axis_point[2] + offset[2]};
  h.optical = Mul(head, Vec3{0, 0, 1});
  h.scan = Mul(head, Vec3{0, 1, 0});
  h.line = Mul(head, Vec3{std::cos(m.twist_rad), std::sin(m.twist_rad), 0});
  return h;
}

std::vector<double> PixelTangents(const Config& config) {
  std::vector<double> t(config.width);
  for (int u = 0; u < config.width; ++u) t[u] = config.PixelTangent(u);
  return t;
}

std::vector<int> DebugColumns(const Config& config) {
  std::vector<int> columns;
  if (config.debug_column_stride <= 0) return columns;
  for (int u = 0; u < config.width; u += config.debug_column_stride) columns.push_back(u);
  if (columns.back() != config.width - 1) columns.push_back(config.width - 1);
  return columns;
}

}  // namespace ssb
