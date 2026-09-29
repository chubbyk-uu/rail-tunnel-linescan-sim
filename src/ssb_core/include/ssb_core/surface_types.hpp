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
struct CrackSegment { float x0, q0, x1, q1, r0, r1; };
static_assert(sizeof(CrackSegment) == 24);
}
