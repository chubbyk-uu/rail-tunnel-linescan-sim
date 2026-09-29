#pragma once
#include <optix.h>

// Row frame: world translated so the row's optical centre has x = 0. The tunnel is
// x-invariant, so float ray origins keep sub-micrometre precision at any track position;
// absolute x is restored in double from origin_x.
struct DeviceRow {
  double origin_x;
  float origin_y, origin_z;
  float optical[3];
  float line[3];
};

struct LaunchParams {
  OptixTraversableHandle handle;
  const DeviceRow* rows;
  const float* tangents;
  unsigned width, row_count;
  double radius, axis_z, x_min, x_max;
  unsigned char* pixels;
  unsigned* invalid;             // per row: rays that missed the wall or left its x range
  const int* debug_slot;         // per column: debug slot or -1
  unsigned debug_count;
  double* debug_hits;            // [row][slot][x, q]
};
