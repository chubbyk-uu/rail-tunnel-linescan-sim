#include "ssb_core/config.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "ssb_core/sha256.hpp"

namespace ssb {
namespace {

constexpr double kPi = 3.14159265358979323846;
double Deg(double d) { return d * kPi / 180.0; }

YAML::Node Require(const YAML::Node& parent, const std::string& key, const std::string& path) {
  const YAML::Node node = parent[key];
  if (!node) throw std::runtime_error("config: missing " + path + key);
  return node;
}

template <class T>
T Get(const YAML::Node& parent, const std::string& key, const std::string& path) {
  try {
    return Require(parent, key, path).as<T>();
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("config: bad value for " + path + key + ": " + e.what());
  }
}

void Check(bool ok, const std::string& what) {
  if (!ok) throw std::runtime_error("config: " + what);
}

bool Finite(std::initializer_list<double> values) {
  for (double v : values)
    if (!std::isfinite(v)) return false;
  return true;
}

}  // namespace

Config Config::Load(const std::filesystem::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("config: cannot open " + path.string());
  std::stringstream buffer;
  buffer << in.rdbuf();
  return Parse(buffer.str());
}

Config Config::Parse(const std::string& text) {
  const YAML::Node root = YAML::Load(text);
  Check(Get<std::string>(root, "schema", "") == "ssb.config.v1", "unsupported schema");
  Config c;
  c.source_text = text;
  c.source_sha256 = Sha256Hex(text.data(), text.size());

  const auto tunnel = Require(root, "tunnel", "");
  c.tunnel_radius_m = Get<double>(tunnel, "radius_m", "tunnel.");
  c.tunnel_axis_z_m = Get<double>(tunnel, "axis_z_m", "tunnel.");
  c.tunnel_x_min_m = Get<double>(tunnel, "x_min_m", "tunnel.");
  c.tunnel_x_max_m = Get<double>(tunnel, "x_max_m", "tunnel.");

  const auto enc = Require(root, "scan_encoder", "");
  c.scan_ppr = Get<int>(enc, "ppr", "scan_encoder.");
  c.scan_edges_per_cycle = Get<int>(enc, "edges_per_cycle", "scan_encoder.");

  const auto res = Require(root, "rescaler", "");
  c.rescale_multiply = Get<int>(res, "multiply", "rescaler.");
  c.rescale_divide = Get<int>(res, "divide", "rescaler.");
  c.rescale_max_period_s = Get<double>(res, "max_period_s", "rescaler.");

  const auto cam = Require(root, "camera", "");
  c.width = Get<int>(cam, "width", "camera.");
  c.pixel_pitch_m = Get<double>(cam, "pixel_pitch_m", "camera.");
  c.fov_at_nominal_m = Get<double>(cam, "fov_at_nominal_m", "camera.");
  c.nominal_distance_m = Get<double>(cam, "nominal_distance_m", "camera.");
  c.exposure_s = Get<double>(cam, "exposure_s", "camera.");
  c.trigger_delay_s = Get<double>(cam, "trigger_delay_s", "camera.");
  c.max_line_rate_hz = Get<double>(cam, "max_line_rate_hz", "camera.");

  const auto gate = Require(root, "gate", "");
  c.gate_start_rad = Deg(Get<double>(gate, "start_deg", "gate."));
  c.gate_end_rad = Deg(Get<double>(gate, "end_deg", "gate."));

  const auto odo = Require(root, "odometer", "");
  c.odo_ppr = Get<int>(odo, "ppr", "odometer.");
  c.odo_edges_per_cycle = Get<int>(odo, "edges_per_cycle", "odometer.");
  c.odo_gear_ratio = Get<double>(odo, "gear_ratio", "odometer.");

  const auto motion = Require(root, "motion", "");
  c.line_rate_hz = Get<double>(motion, "line_rate_hz", "motion.");
  c.advance_per_rev_m = Get<double>(motion, "advance_per_rev_m", "motion.");
  c.start_theta_rad = Deg(Get<double>(motion, "start_theta_deg", "motion."));
  c.start_x_m = Get<double>(motion, "start_x_m", "motion.");
  c.sample_period_s = Get<double>(motion, "sample_period_s", "motion.");
  for (const auto& knot : Require(motion, "profile", "motion.")) {
    Check(knot.IsSequence() && knot.size() == 2, "motion.profile entries must be [time_s, factor]");
    c.profile.push_back({knot[0].as<double>(), knot[1].as<double>()});
  }

  const auto truth = Require(root, "truth", "");
  c.truth.wheel_diameter_m = Get<double>(truth, "wheel_diameter_m", "truth.");
  c.truth.scan_encoder_zero_rad = Deg(Get<double>(truth, "scan_encoder_zero_deg", "truth."));
  c.truth.gate_start_offset_rad = Deg(Get<double>(truth, "gate_start_offset_deg", "truth."));
  c.truth.gate_end_offset_rad = Deg(Get<double>(truth, "gate_end_offset_deg", "truth."));
  c.truth.head_mount_x_m = Get<double>(truth, "head_mount_x_m", "truth.");
  const auto mount = Require(truth, "mount", "truth.");
  auto& m = c.truth.mount;
  m.e_m = Get<double>(mount, "e_m", "truth.mount.");
  m.tangential_m = Get<double>(mount, "tangential_m", "truth.mount.");
  m.dy_m = Get<double>(mount, "dy_m", "truth.mount.");
  m.dz_m = Get<double>(mount, "dz_m", "truth.mount.");
  m.tilt_y_rad = Get<double>(mount, "tilt_y_rad", "truth.mount.");
  m.tilt_z_rad = Get<double>(mount, "tilt_z_rad", "truth.mount.");
  m.twist_rad = Get<double>(mount, "twist_rad", "truth.mount.");

  const auto cal = Require(root, "calibration", "");
  c.calibration.wheel_diameter_m = Get<double>(cal, "wheel_diameter_m", "calibration.");
  c.calibration.radius_m = Get<double>(cal, "radius_m", "calibration.");
  c.calibration.head_mount_x_m = Get<double>(cal, "head_mount_x_m", "calibration.");

  const auto render = Require(root, "render", "");
  c.batch_rows = Get<int>(render, "batch_rows", "render.");
  c.debug_column_stride = Get<int>(render, "debug_column_stride", "render.");
  c.debug_delay_per_batch_s = Get<double>(render, "debug_delay_per_batch_s", "render.");

  const auto storage = Require(root, "storage", "");
  c.block_rows = Get<int>(storage, "block_rows", "storage.");
  c.write_queue_bytes = Get<size_t>(storage, "write_queue_bytes", "storage.");

  c.Validate();
  return c;
}

void Config::Validate() const {
  Check(Finite({tunnel_radius_m, tunnel_axis_z_m, tunnel_x_min_m, tunnel_x_max_m, rescale_max_period_s,
                pixel_pitch_m, fov_at_nominal_m, nominal_distance_m, exposure_s, trigger_delay_s,
                max_line_rate_hz, gate_start_rad, gate_end_rad, odo_gear_ratio, line_rate_hz,
                advance_per_rev_m, start_theta_rad, start_x_m, sample_period_s, debug_delay_per_batch_s}),
        "non-finite value");
  Check(tunnel_radius_m > 0 && tunnel_x_max_m > tunnel_x_min_m, "invalid tunnel");
  Check(scan_ppr > 0 && scan_edges_per_cycle > 0, "invalid scan encoder");
  Check(rescale_multiply > 0 && rescale_divide > 0 && rescale_max_period_s > 0, "invalid rescaler");
  Check(width > 1 && pixel_pitch_m > 0 && fov_at_nominal_m > 0 && nominal_distance_m > 0, "invalid camera");
  Check(exposure_s > 0 && trigger_delay_s >= 0 && max_line_rate_hz > 0, "invalid camera timing");
  Check(exposure_s < 1.0 / max_line_rate_hz, "exposure longer than the line period limit");
  Check(gate_end_rad > gate_start_rad && gate_end_rad - gate_start_rad < 2 * kPi, "invalid gate arc");
  Check(odo_ppr > 0 && odo_edges_per_cycle > 0 && odo_gear_ratio > 0, "invalid odometer");
  Check(line_rate_hz > 0 && advance_per_rev_m > 0 && sample_period_s > 0, "invalid motion");
  Check(profile.size() >= 2 && profile.front()[0] == 0, "profile must start at t=0 with at least two knots");
  for (size_t i = 0; i < profile.size(); ++i) {
    Check(std::isfinite(profile[i][0]) && std::isfinite(profile[i][1]) && profile[i][1] >= 0,
          "invalid profile knot");
    if (i) Check(profile[i][0] > profile[i - 1][0], "profile times must increase");
  }
  Check(truth.wheel_diameter_m > 0 && calibration.wheel_diameter_m > 0 && calibration.radius_m > 0,
        "invalid wheel or radius");
  const auto& m = truth.mount;
  Check(Finite({truth.scan_encoder_zero_rad, truth.gate_start_offset_rad, truth.gate_end_offset_rad,
                truth.head_mount_x_m, m.e_m, m.tangential_m, m.dy_m, m.dz_m, m.tilt_y_rad, m.tilt_z_rad,
                m.twist_rad, calibration.head_mount_x_m}),
        "non-finite truth or calibration");
  Check(batch_rows > 0 && batch_rows <= 16384, "render.batch_rows must be in [1, 16384]");
  Check(debug_column_stride >= 0 && debug_column_stride < width, "invalid debug column stride (0 disables)");
  Check(debug_delay_per_batch_s >= 0, "negative debug delay");
  Check(block_rows > 0 && write_queue_bytes >= size_t(width) * block_rows, "write queue smaller than a block");
}

double Config::NominalOmega() const { return 2 * kPi * line_rate_hz / RowsPerRev(); }

nlohmann::json Config::ObservableJson() const {
  nlohmann::json j;
  j["schema"] = "ssb.observable_config.v1";
  j["tunnel"] = {{"x_min_m", tunnel_x_min_m}, {"x_max_m", tunnel_x_max_m}};
  j["scan_encoder"] = {{"ppr", scan_ppr}, {"edges_per_cycle", scan_edges_per_cycle}};
  j["rescaler"] = {{"multiply", rescale_multiply}, {"divide", rescale_divide},
                   {"max_period_s", rescale_max_period_s}};
  j["camera"] = {{"width", width}, {"pixel_pitch_m", pixel_pitch_m}, {"fov_at_nominal_m", fov_at_nominal_m},
                 {"nominal_distance_m", nominal_distance_m}, {"exposure_s", exposure_s},
                 {"trigger_delay_s", trigger_delay_s}, {"max_line_rate_hz", max_line_rate_hz},
                 {"focal_length_m", FocalLength()}};
  j["gate"] = {{"start_rad", gate_start_rad}, {"end_rad", gate_end_rad}};
  j["odometer"] = {{"ppr", odo_ppr}, {"edges_per_cycle", odo_edges_per_cycle}, {"gear_ratio", odo_gear_ratio}};
  j["motion"] = {{"line_rate_hz", line_rate_hz}, {"advance_per_rev_m", advance_per_rev_m},
                 {"start_x_m", start_x_m}, {"sample_period_s", sample_period_s}, {"profile", profile}};
  j["calibration"] = {{"wheel_diameter_m", calibration.wheel_diameter_m}, {"radius_m", calibration.radius_m},
                      {"head_mount_x_m", calibration.head_mount_x_m}};
  j["render"] = {{"batch_rows", batch_rows}, {"debug_column_stride", debug_column_stride}};
  j["storage"] = {{"block_rows", block_rows}};
  j["derived"] = {{"counts_per_rev", CountsPerRev()}, {"rows_per_rev", RowsPerRev()}};
  return j;
}

nlohmann::json Config::TruthJson() const {
  const auto& m = truth.mount;
  return {{"schema", "ssb.truth.v1"},
          {"tunnel", {{"radius_m", tunnel_radius_m}, {"axis_z_m", tunnel_axis_z_m}}},
          {"start_theta_rad", start_theta_rad},
          {"wheel_diameter_m", truth.wheel_diameter_m},
          {"scan_encoder_zero_rad", truth.scan_encoder_zero_rad},
          {"gate_start_offset_rad", truth.gate_start_offset_rad},
          {"gate_end_offset_rad", truth.gate_end_offset_rad},
          {"head_mount_x_m", truth.head_mount_x_m},
          {"mount",
           {{"e_m", m.e_m}, {"tangential_m", m.tangential_m}, {"dy_m", m.dy_m}, {"dz_m", m.dz_m},
            {"tilt_y_rad", m.tilt_y_rad}, {"tilt_z_rad", m.tilt_z_rad}, {"twist_rad", m.twist_rad}}},
          {"config_sha256", source_sha256}};
}

ProfileState EvaluateProfile(const std::vector<std::array<double, 2>>& k, double t) {
  if (k.empty() || t < k.front()[0]) throw std::invalid_argument("profile evaluated before its start");
  double integral = 0;
  for (size_t i = 0; i + 1 < k.size(); ++i) {
    const double ta = k[i][0], tb = k[i + 1][0], fa = k[i][1], fb = k[i + 1][1], H = tb - ta;
    if (t >= tb) {
      integral += H * 0.5 * (fa + fb);
      continue;
    }
    const double u = (t - ta) / H, u3 = u * u * u;
    const double S = u3 * (10 + u * (-15 + 6 * u));
    const double I = u3 * u * (2.5 + u * (-3 + u));  // integral of S from 0 to u
    return {fa + (fb - fa) * S, integral + H * (fa * u + (fb - fa) * I)};
  }
  return {k.back()[1], integral + (t - k.back()[0]) * k.back()[1]};
}

}  // namespace ssb
