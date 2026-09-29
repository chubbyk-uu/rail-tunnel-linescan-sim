#include <optix.h>

#include "launch_params.h"
#include "ssb_core/texture.hpp"

extern "C" {
__constant__ LaunchParams params;
}

namespace {
__device__ unsigned Lo(double v) { return static_cast<unsigned>(__double_as_longlong(v) & 0xffffffffull); }
__device__ unsigned Hi(double v) { return static_cast<unsigned>(static_cast<unsigned long long>(__double_as_longlong(v)) >> 32); }
__device__ double Join(unsigned lo, unsigned hi) {
  return __longlong_as_double(static_cast<long long>((static_cast<unsigned long long>(hi) << 32) | lo));
}
}  // namespace

// Analytic cylinder about (y, z) = (0, axis_z); the ray starts inside it.
extern "C" __global__ void __intersection__tunnel() {
  const float3 o = optixGetObjectRayOrigin(), d = optixGetObjectRayDirection();
  const double oy = o.y, oz = o.z - params.axis_z, dy = d.y, dz = d.z;
  const double a = dy * dy + dz * dz;
  if (a <= 0) return;
  const double b = 2 * (oy * dy + oz * dz), c = oy * oy + oz * oz - params.radius * params.radius;
  const double disc = b * b - 4 * a * c;
  if (disc < 0) return;
  const double q = -0.5 * (b + copysign(sqrt(disc), b));
  const double r1 = q / a, r2 = q != 0 ? c / q : r1;
  const double t = fmax(r1, r2);
  if (t >= optixGetRayTmin() && t <= optixGetRayTmax())
    optixReportIntersection(static_cast<float>(t), 0, Lo(t), Hi(t));
}

extern "C" __global__ void __closesthit__tunnel() {
  optixSetPayload_0(optixGetAttribute_0());
  optixSetPayload_1(optixGetAttribute_1());
  optixSetPayload_2(1u);
}

extern "C" __global__ void __miss__primary() { optixSetPayload_2(0u); }

extern "C" __global__ void __raygen__scan() {
  const uint3 idx = optixGetLaunchIndex();
  const unsigned u = idx.x, r = idx.y;
  const DeviceRow row = params.rows[r];
  const float tan_u = params.tangents[u];
  float3 d = make_float3(row.optical[0] + tan_u * row.line[0], row.optical[1] + tan_u * row.line[1],
                         row.optical[2] + tan_u * row.line[2]);
  const float inv = rsqrtf(d.x * d.x + d.y * d.y + d.z * d.z);
  d = make_float3(d.x * inv, d.y * inv, d.z * inv);
  unsigned lo = 0, hi = 0, hit = 0;
  optixTrace(params.handle, make_float3(0.f, row.origin_y, row.origin_z), d, 0.f, 1e3f, 0.f, 255,
             OPTIX_RAY_FLAG_DISABLE_ANYHIT, 0, 1, 0, lo, hi, hit);
  unsigned char code = 0;
  bool valid = false;
  double x = 0, q = 0;
  if (hit) {
    const double t = Join(lo, hi);
    x = row.origin_x + t * d.x;
    const double y = row.origin_y + t * d.y, z = row.origin_z + t * d.z;
    q = params.radius * atan2(y, z - params.axis_z);
    valid = x >= params.x_min && x <= params.x_max;
    if (valid) code = ssb::AlbedoCode(ssb::WallAlbedo(x, q));
  }
  if (!valid) atomicAdd(params.invalid + r, 1u);
  params.pixels[static_cast<size_t>(r) * params.width + u] = code;
  const int slot = params.debug_slot[u];
  if (slot >= 0) {
    double* out = params.debug_hits + (static_cast<size_t>(r) * params.debug_count + slot) * 2;
    out[0] = valid ? x : nan("");
    out[1] = valid ? q : nan("");
  }
}
