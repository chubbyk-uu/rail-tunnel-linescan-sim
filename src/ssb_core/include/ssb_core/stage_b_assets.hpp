#pragma once
#include <set>
#include <map>
#include "ssb_core/camera_model.hpp"
#include "ssb_core/surface_types.hpp"
#include "ssb_core/surface_recipe.hpp"

namespace ssb {
struct OpticalVertex { float x, y, z; };
struct OpticalTriangle { unsigned a, b, c; };
struct StageBAssets {
  explicit StageBAssets(const Config& config);
  std::filesystem::path path, surface_path, defect_path;
  nlohmann::json scene, surface, defects;
  std::unique_ptr<SurfaceRecipe> recipe;
  std::string scene_hash, surface_hash, defect_hash;
  unsigned core=0, gutter=0, side=0, nx=0, nq=0, pixel_x=0, pixel_q=0;
  double x0=0, q0=0, dx=0, dq=0, period=0, max_radius=0;
  double tunnel_radius=0, tunnel_axis_z=0;   // cached from the surface manifest (Footprint hot path)
  unsigned area_samples=2, time_samples=2, light_samples=3;
  bool light_enabled=true, shadows=true;
  bool convex_panel_visibility=false, adaptive_area=false;
  // Full-ray area pattern: 0 = area_samples x area_samples grid per exposure sample; otherwise
  // the rank-1 lattice generator of an area_samples-point N-rooks pattern (see scan.cu).
  unsigned area_rooks=0;
  // Complex crack pixels (integrated_cracks): N-rooks rays per exposure sample and its generator.
  unsigned crack_area_samples=64, crack_area_rooks=19;
  bool integrated_cracks=false;
  // Background texture taps per axis over the pixel footprint (integrated path only).
  unsigned texture_footprint_samples=1;
  bool texture_prefilter=false;
  double indirect_fill=0;
  double response_gain=1, lamp_length=.3, footprint_x=1.2, footprint_q=.12;
  double lamp_tangential=.2, lamp_radial=.025, lamp_axial=0, lamp_width=0;
  size_t gpu_budget=0, cpu_budget=0, tile_bytes=0, cache_slots=0;
  std::vector<OpticalVertex> vertices;
  std::vector<OpticalTriangle> triangles;
  std::vector<unsigned> face_material;
  std::vector<unsigned> critical_edges;
  // Joint filler (material 2): mean-normalised albedo detail tiled by wall (x, q).
  std::vector<uint16_t> filler;
  unsigned filler_width=0, filler_height=0;
  double filler_pitch=0, filler_scale=1, filler_mean=0, filler_roughness=.9;
  std::string filler_hash;
  // Groove walls/floor (material 1) and the dark gasket/void behind the contact gap (3).
  double groove_albedo=.12, groove_detail_contrast=.3, gap_albedo=.02;
  // Crack appearance: legacy scenes (no crack_optics) keep the flat 0.035 opening, no edge band.
  double crack_interior=-1, crack_interior_variation=0, crack_edge_band=0, crack_edge_darkening=0;
  // crack_optics.model "cavity_v2": interior from the defects' per-segment effective visible
  // depth (depths.bin, [d0,d1] metres per segment) through a slot-cavity reflectance, flat
  // cross-section; lip band darkening scaled by the interior darkening.
  bool crack_cavity=false;
  std::vector<float> crack_depths;
  std::vector<CrackSegment> segments;
  std::vector<unsigned> offsets, indices;
  double crack_x0=0, crack_q0=0, crack_cell=0;
  unsigned crack_nx=0, crack_nq=0;
  std::vector<SurfaceTexel> ReadTile(unsigned index) const;
  // Conservative texture window of one camera line: x tile columns and a periodic q interval.
  struct FootprintWindow { int ix0, ix1; double centre_q, half_q; };
  FootprintWindow Window(const HeadPose& h, double pixel_tangent_step, double max_tangent) const;
  std::set<unsigned> Footprint(const HeadPose& h, double pixel_tangent_step, double max_tangent) const;
  bool CpuHit(const HeadPose& h, double tangent, double scan_tangent, double* x, double* q) const;
};
}
