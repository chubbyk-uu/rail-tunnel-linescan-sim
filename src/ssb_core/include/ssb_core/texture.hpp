#pragma once
// Stage A procedural wall albedo in tunnel coordinates (x along the axis, q = R * theta).
// 50 mm checker plus 1 mm hashed noise. Shared by the GPU kernel and the host self-check;
// the independent reference lives in ssb_tools.
#include <cmath>
#include <cstdint>

#ifdef __CUDACC__
#define SSB_HD __host__ __device__
#else
#define SSB_HD
#endif

namespace ssb {

SSB_HD inline uint64_t HashCell(int64_t a, int64_t b) {
  uint64_t z = static_cast<uint64_t>(a) * 0x9E3779B97F4A7C15ull ^ static_cast<uint64_t>(b) * 0xC2B2AE3D27D4EB4Full;
  z += 0x9E3779B97F4A7C15ull;
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
  return z ^ (z >> 31);
}

SSB_HD inline double WallAlbedo(double x, double q) {
  const int64_t cx = static_cast<int64_t>(floor(x / 0.05)), cq = static_cast<int64_t>(floor(q / 0.05));
  const double base = ((cx + cq) & 1) ? 0.65 : 0.35;
  const int64_t ix = static_cast<int64_t>(floor(x / 0.001)), iq = static_cast<int64_t>(floor(q / 0.001));
  const double noise = static_cast<double>(HashCell(ix, iq) >> 40) / 16777216.0 - 0.5;
  return base + 0.2 * noise;
}

SSB_HD inline unsigned char AlbedoCode(double albedo) {
  const double v = rint(albedo * 255.0);
  return static_cast<unsigned char>(v < 0 ? 0 : (v > 255 ? 255 : v));
}

}  // namespace ssb
#undef SSB_HD
