#pragma once
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "ssb_core/config.hpp"
#include "ssb_core/records.hpp"
#include "ssb_core/renderer.hpp"

namespace ssb {

struct PipelineOptions {
  std::filesystem::path session;   // must not exist yet
  std::vector<std::string> argv;   // recorded in provenance
  std::string pose_source;         // e.g. "kinematic", "file:<path>", "gazebo"
  // Content identity of every input, {name: {path, sha256}}; written to provenance.
  nlohmann::json inputs = nlohmann::json::object();
  // Sim time the planned motion ends at; NaN when the source has no plan (archived stream).
  double planned_end_s = std::numeric_limits<double>::quiet_NaN();
};

// {path, sha256} of a file, for PipelineOptions::inputs.
nlohmann::json FileIdentity(const std::filesystem::path& path);

// Pose stream in, session out (DESIGN.md §8.2-§8.3). Push never waits for imaging:
// samples are queued without bound (a 20 m run is a few MB); timing, rendering and
// writing run on their own threads and may lag arbitrarily. The row-job queue before
// rendering and the pixel queue after it are bounded: they hold back timing expansion
// and rendering, never the producer, whose low-rate pose samples stay buffered.
class Pipeline {
 public:
  Pipeline(const Config& config, std::unique_ptr<RowRenderer> renderer, const PipelineOptions& options);
  ~Pipeline();
  Pipeline(const Pipeline&) = delete;
  Pipeline& operator=(const Pipeline&) = delete;

  // After a worker failure input is discarded; Wait reports the original error.
  void Push(const PoseSample& sample);
  // Producer failure drains accepted input and preserves indices, then marks failed.
  void Finish(const std::string& producer_error = "");
  // Contact producer has observed target distance and parked; no world pose input.
  // A plain Finish on a distance task drains data but leaves motion incomplete.
  void FinishDistanceMotion(double estimated_distance_m, double estimated_speed_m_s);
  // Low-rate UI snapshot; does not expose truth or copy image buffers.
  nlohmann::json Progress() const;
  // Ends input (idempotent Finish), then blocks until everything is on disk. Rethrows the
  // first worker error after marking the session failed.
  nlohmann::json Wait();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ssb
