#pragma once
#include <memory>
#include <filesystem>
#include <vector>
#include <nlohmann/json.hpp>
#include "ssb_core/surface_types.hpp"

namespace ssb {
// Compact, deterministic quilt replay. Coordinates are metres on cylinder (x,q).
struct RecipePatch {
  int left, top, source;
  int m00, m01, m10, m11;
  double ox, oq;
};
struct RecipeSource { const SurfaceTexel* data; int width, height; double native; };
// Optional low-frequency albedo modulation over the wall (value = code/scale), x clamped, q periodic.
struct RecipeMacro { const uint16_t* data=nullptr; int width=0, height=0; double x0=0, q0=0, pitch=0, scale=1; };
struct RecipeGrid {
  unsigned core, gutter, nx, pixel_x, pixel_q;
  double x0,q0,dx,dq,period,guide_x0,guide_q0,guide_step;
  unsigned patch_side;
};
class SurfaceRecipe {
 public:
  SurfaceRecipe(const std::filesystem::path& surface_path,const nlohmann::json& surface);
  ~SurfaceRecipe();
  SurfaceRecipe(const SurfaceRecipe&)=delete;
  SurfaceRecipe& operator=(const SurfaceRecipe&)=delete;
  std::vector<int> Select(unsigned tile) const;
  std::vector<SurfaceTexel> Generate(unsigned tile) const;
  RecipeGrid grid{};
  std::vector<RecipeSource> sources;
  std::vector<RecipePatch> patches;
  RecipeMacro macro;
  const unsigned char* alpha=nullptr;
  size_t alpha_bytes=0,source_bytes=0;
 private:
  struct Mapping;
  std::vector<std::unique_ptr<Mapping>> maps_;
};

// Allocated source data is separate from the renderer's generated-tile cache.
class CudaSurfaceRecipe {
 public:
  explicit CudaSurfaceRecipe(const SurfaceRecipe& recipe);
  ~CudaSurfaceRecipe();
  CudaSurfaceRecipe(const CudaSurfaceRecipe&)=delete;
  void Generate(unsigned tile,SurfaceTexel* output,void* stream);
  // Generates several tiles with one launch and one synchronisation (per-call CUDA overhead
  // dominates single-tile generation); results are identical to per-tile Generate.
  void Generate(const std::vector<std::pair<unsigned,SurfaceTexel*>>& jobs,void* stream);
  size_t Bytes() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
