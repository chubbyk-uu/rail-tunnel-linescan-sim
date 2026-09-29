#pragma once
#include <array>
#include <filesystem>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace ssb {

// Head installation offsets relative to nominal (DESIGN.md §4.4). Zero is nominal.
struct MountOffsets {
  double e_m = 0;            // optical centre along the optical axis
  double tangential_m = 0;   // optical centre along the scan direction
  double dy_m = 0, dz_m = 0; // rotation axis relative to the tunnel axis
  double tilt_y_rad = 0, tilt_z_rad = 0;
  double twist_rad = 0;      // line rotation about the optical axis
};

struct Config {
  // Tunnel (scene truth; the nominal radius for reconstruction is calibration.radius_m).
  double tunnel_radius_m = 0, tunnel_axis_z_m = 0, tunnel_x_min_m = 0, tunnel_x_max_m = 0;
  int scan_ppr = 0, scan_edges_per_cycle = 0;
  int rescale_multiply = 0, rescale_divide = 0;
  double rescale_max_period_s = 0;
  int width = 0;
  double pixel_pitch_m = 0, fov_at_nominal_m = 0, nominal_distance_m = 0;
  double exposure_s = 0, trigger_delay_s = 0, max_line_rate_hz = 0;
  double gate_start_rad = 0, gate_end_rad = 0;
  int odo_ppr = 0, odo_edges_per_cycle = 0;
  double odo_gear_ratio = 0;
  double line_rate_hz = 0, advance_per_rev_m = 0;
  double start_theta_rad = 0, start_x_m = 0, sample_period_s = 0;
  std::vector<std::array<double, 2>> profile;  // [time_s, speed factor]

  struct Truth {
    double wheel_diameter_m = 0, scan_encoder_zero_rad = 0;
    double gate_start_offset_rad = 0, gate_end_offset_rad = 0, head_mount_x_m = 0;
    MountOffsets mount;
  } truth;
  struct Calibration {
    double wheel_diameter_m = 0, radius_m = 0, head_mount_x_m = 0;
  } calibration;

  int batch_rows = 0, debug_column_stride = 0, max_queued_batches = 0;
  double debug_delay_per_batch_s = 0;
  int block_rows = 0;
  size_t write_queue_bytes = 0;
  std::filesystem::path optical_scene;  // optional scene truth; not a reconstruction input

  // Raw bytes and hash of the file this config was loaded from.
  std::string source_text, source_sha256;

  static Config Load(const std::filesystem::path& path);
  static Config Parse(const std::string& yaml_text);
  void Validate() const;

  double CountsPerRev() const { return double(scan_ppr) * scan_edges_per_cycle; }
  double RowsPerRev() const { return CountsPerRev() * rescale_multiply / rescale_divide; }
  double FocalLength() const { return pixel_pitch_m * nominal_distance_m * width / fov_at_nominal_m; }
  double NominalOmega() const;  // scan axis rad/s at speed factor 1
  // Tangent of the angle between pixel u's ray and the optical axis, along the line.
  double PixelTangent(int u) const { return (u - 0.5 * (width - 1)) * pixel_pitch_m / FocalLength(); }

  // Everything except `truth`: what a session exposes to reconstruction (config/).
  nlohmann::json ObservableJson() const;
  // Only `truth` plus derived true quantities (evaluation/).
  nlohmann::json TruthJson() const;
};

// Speed factor and its time integral for the quintic-smootherstep knot profile.
struct ProfileState {
  double factor = 0, integral = 0;
};
ProfileState EvaluateProfile(const std::vector<std::array<double, 2>>& knots, double t);

}  // namespace ssb
