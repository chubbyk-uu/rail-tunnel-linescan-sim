#pragma once
#include <filesystem>
#include <memory>

#include "ssb_core/config.hpp"
#include "ssb_core/renderer.hpp"

namespace ssb {

// OptiX line-scan backend. Stage A scene: analytic tunnel cylinder with the procedural
// wall albedo; no lighting. There is no fallback backend: construction throws if
// CUDA/OptiX cannot be initialised.
class OptixRenderer final : public RowRenderer {
 public:
  OptixRenderer(const Config& config, const std::filesystem::path& ptx, size_t capacity);
  ~OptixRenderer() override;
  OptixRenderer(const OptixRenderer&) = delete;
  OptixRenderer& operator=(const OptixRenderer&) = delete;

  void Render(const std::vector<RowJob>& rows, std::vector<uint8_t>& pixels, std::vector<double>& hits) override;
  nlohmann::json Describe() const override;
  nlohmann::json SelfCheck() override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Installed PTX, overridable with SSB_PTX for tests run from the build tree.
std::filesystem::path DefaultPtxPath();

}  // namespace ssb
