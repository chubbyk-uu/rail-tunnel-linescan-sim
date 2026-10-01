#include <gtest/gtest.h>
#include <unistd.h>
#include <chrono>
#include <fstream>
#include "ssb_core/kinematic.hpp"
#include "ssb_core/pipeline.hpp"

namespace {
class FailingRenderer : public ssb::RowRenderer {
 public:
  explicit FailingRenderer(int width) : width_(width) {}
  void Render(const std::vector<ssb::RowJob>& rows, std::vector<uint8_t>& pixels,
              std::vector<double>& hits) override {
    if (++calls_ == 2) throw std::runtime_error("injected rendering failure");
    pixels.assign(rows.size()*width_, 42);
    hits.clear();
  }
  nlohmann::json Describe() const override { return {{"test_backend", true}}; }
  nlohmann::json SelfCheck() override { return {{"passed",true}}; }
 private:
  int width_, calls_=0;
};
}

TEST(PipelineFailure, StopsLocalPoseDrainAndNeverRepeatsTableRecords) {
  auto c=ssb::Config::Load(std::string(SSB_CONFIG_DIR)+"/stage_a.yaml");
  c.profile={{0,1},{100,1}};
  c.batch_rows=64;c.max_queued_batches=1;c.debug_column_stride=0;
  const auto samples=ssb::KinematicSource(c).Sample();
  const auto root=std::filesystem::temp_directory_path()/("ssb_failure_"+std::to_string(getpid()));
  std::filesystem::remove_all(root);
  {
    ssb::Pipeline p(c,std::make_unique<FailingRenderer>(c.width),{root,{"test"},"kinematic"});
    for(const auto& sample:samples) p.Push(sample);
    p.Finish();
    EXPECT_THROW(p.Wait(),std::runtime_error);
    // Input after failure is discarded, including after Finish().
    p.Push(samples.back());
  }
  nlohmann::json summary;std::ifstream(root/"session.json")>>summary;
  EXPECT_EQ(summary.at("status"),"failed");
  EXPECT_EQ(summary.at("error"),"injected rendering failure");
  const auto poses=std::filesystem::file_size(root/"evaluation/pose_stream.bin")/sizeof(ssb::PoseSample);
  EXPECT_LT(poses,samples.size()/10);
  const auto path=root/"metadata/scan_edges.bin";
  std::vector<ssb::EdgeRecord> edges(std::filesystem::file_size(path)/sizeof(ssb::EdgeRecord));
  std::ifstream(path,std::ios::binary).read(reinterpret_cast<char*>(edges.data()),edges.size()*sizeof(edges[0]));
  for(size_t i=1;i<edges.size();++i) EXPECT_GT(edges[i].t,edges[i-1].t);
  EXPECT_FALSE(std::filesystem::exists(root/"raw/index.json"));
  std::filesystem::remove_all(root);
}
