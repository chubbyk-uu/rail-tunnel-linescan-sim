#pragma once
#include <filesystem>
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
};

// Pose stream in, session out (DESIGN.md §8.2-§8.3). Push never waits for imaging:
// samples are queued without bound (a 20 m run is a few MB); timing, rendering and
// writing run on their own threads and may lag arbitrarily. Only the write queue is
// bounded, and it holds back rendering, never the producer.
class Pipeline {
 public:
  Pipeline(const Config& config, std::unique_ptr<RowRenderer> renderer, const PipelineOptions& options);
  ~Pipeline();
  Pipeline(const Pipeline&) = delete;
  Pipeline& operator=(const Pipeline&) = delete;

  void Push(const PoseSample& sample);
  void Finish();
  // Blocks until everything is on disk; returns the session summary. Rethrows the
  // first worker error after marking the session failed.
  nlohmann::json Wait();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ssb
