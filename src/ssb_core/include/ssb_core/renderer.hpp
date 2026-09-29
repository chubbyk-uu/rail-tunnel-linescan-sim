#pragma once
#include <cstdint>
#include <vector>

#include <nlohmann/json.hpp>

#include "ssb_core/timing.hpp"

namespace ssb {

// Imaging backend. Each pixel depends only on its own row job, so results never
// depend on how rows are grouped into batches.
class RowRenderer {
 public:
  virtual ~RowRenderer() = default;
  // pixels: rows.size() x width. hits: rows.size() x debug columns x {x, q}, true
  // hit coordinates of the archived debug columns (evaluation only).
  virtual void Render(const std::vector<RowJob>& rows, std::vector<uint8_t>& pixels,
                      std::vector<double>& hits) = 0;
  virtual nlohmann::json Describe() const = 0;
  virtual nlohmann::json EvaluationAssets() const { return nullptr; }
  // Functional check of the loaded backend against an analytic answer; throws on failure.
  virtual nlohmann::json SelfCheck() = 0;
};

}  // namespace ssb
