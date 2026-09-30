#pragma once
#include <optix.h>
#include "ssb_core/surface_types.hpp"

// Row frame: world translated so the row's optical centre has x = 0. The tunnel is
// x-invariant, so float ray origins keep sub-micrometre precision at any track position;
// absolute x is restored in double from origin_x.
struct DeviceRow {
  double origin_x;
  float origin_y, origin_z;
  float optical[3];
  float line[3];
  float scan[3];
  unsigned body_pose;
  double optical_q;  // Per-exposure-frame beam centre, shared by all pixel samples.
};

struct LaunchParams {
  OptixTraversableHandle handle;
  const DeviceRow* rows;
  const float* tangents;
  const float2* pixel_steps; // inverse lens Jacobian (line, perpendicular)
  unsigned width, row_count;
  double radius, axis_z, x_min, x_max;
  unsigned char* pixels;
  unsigned* invalid;             // per row: rays that missed the wall or left its x range
  unsigned* invalid_flags;
  unsigned* invalid_column;
  const int* debug_slot;         // per column: debug slot or -1
  unsigned debug_count;
  double* debug_hits;            // [row][slot][x, q]
  unsigned stage_b, row_stride, area_samples, time_samples, light_samples;
  float pixel_step, response_gain, indirect_fill;
  unsigned calibration_target;
  double target_origin, target_pitch, target_width;
  float target_albedo;
  const ssb::SurfaceTexel* const* tiles;
  unsigned tile_core, tile_gutter, tile_side, tiles_x, tiles_q, pixels_x, pixels_q;
  double tex_x0, tex_q0, tex_dx, tex_dq, tex_period;
  const float3* vertices;
  const uint3* triangles;
  const unsigned* face_material;
  const unsigned* critical_edges;
  const ssb::CrackSegment* cracks;
  const unsigned* crack_offsets;
  const unsigned* crack_indices;
  const float* crack_depths;     // cavity model: [d0, d1] effective visible depth per segment, or null
  double crack_x0, crack_q0, crack_cell;
  unsigned crack_nx, crack_nq;
  unsigned light_enabled, shadows;
  unsigned convex_panel_visibility, adaptive_area, area_rooks, crack_area_samples, crack_area_rooks;
  unsigned integrated_cracks;
  unsigned texture_footprint_samples, texture_prefilter;
  const unsigned short* filler;  // material 2 albedo detail (code/scale), tiled by (x, q)
  unsigned filler_width, filler_height;
  double filler_pitch, filler_scale;
  float filler_mean, filler_roughness;
  float groove_albedo, groove_detail_contrast, gap_albedo;
  float crack_interior, crack_interior_variation, crack_edge_band, crack_edge_darkening;
  float lamp_length, footprint_x, footprint_q, lamp_tangential, lamp_radial, lamp_axial, lamp_width;
};
