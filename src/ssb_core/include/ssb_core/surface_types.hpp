#pragma once
#include <cstdint>

namespace ssb {
// File and device ABI; native little-endian data, one 8-byte texel.
struct SurfaceTexel {
  uint16_t albedo;
  uint8_t roughness, reserved;
  int16_t nx, nq;
};
static_assert(sizeof(SurfaceTexel) == 8);
struct LegacyCrackSegment { float x0, q0, x1, q1, r0, r1; };
struct CrackSegment { double x0, q0, x1, q1; float r0, r1; };
static_assert(sizeof(LegacyCrackSegment) == 24);
static_assert(sizeof(CrackSegment) == 40);
}
