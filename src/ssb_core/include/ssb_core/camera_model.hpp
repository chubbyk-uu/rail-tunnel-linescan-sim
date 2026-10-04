#pragma once
#include <array>
#include <vector>

#include "ssb_core/config.hpp"
#include "ssb_core/records.hpp"

namespace ssb {

using Vec3 = std::array<double, 3>;

// World frame: x along the track, y lateral, z up, rail top at z = 0. The tunnel
// axis is (x, 0, tunnel_axis_z_m). Head frame at theta: optical axis
// (0, sin theta, cos theta) when nominal, scan direction d/dtheta of it, line along x.
struct HeadPose {
  Vec3 origin;   // optical centre
  Vec3 optical;  // unit optical axis
  Vec3 scan;     // unit scan direction (increasing theta)
  Vec3 line;     // unit direction of increasing pixel index
};

// True head pose from a true pose sample and the true mount (DESIGN.md §4.4).
HeadPose TrueHeadPose(const Config& config, const PoseSample& pose);

// Generator-only calibration jig: align the camera independently of its vehicle
// mount to a known cylindrical target. Never used by acquisition/reconstruction.
PoseSample CenteredBenchPose(const Config& config, double axis_x, double theta);

// Per-pixel tangent table, u = 0..width-1.
std::vector<double> PixelTangents(const Config& config);

// Columns whose true hit coordinates are archived for validation (stride, plus last).
std::vector<int> DebugColumns(const Config& config);

}  // namespace ssb
