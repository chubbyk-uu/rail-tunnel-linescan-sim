#pragma once
#include <cmath>
#include "ssb_core/stage_b_assets.hpp"

namespace ssb {
constexpr double kOpticalChunkLength = 2.;
inline double OpticalFrame(double x) { return std::floor(x/kOpticalChunkLength)*kOpticalChunkLength+1.; }
struct LocalOpticalVertex { float x,y,z; };
struct OpticalChunk { double origin_x; unsigned first_triangle,triangle_count; };
struct LocalGeometry {
  std::vector<LocalOpticalVertex> vertices;
  std::vector<OpticalTriangle> triangles;
  std::vector<unsigned> material,critical_edges;
  std::vector<double> primitive_origin_x;
  std::vector<OpticalChunk> chunks;
};
// Clip long triangles at fixed 2 m boundaries. Internal clip edges are not optical defects.
LocalGeometry LocalizeGeometry(const StageBAssets& assets);
}  // namespace ssb
