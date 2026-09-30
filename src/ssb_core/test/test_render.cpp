#include <gtest/gtest.h>
#include <unistd.h>

#include <cmath>
#include <fstream>
#include <iterator>

#include "ssb_core/camera_model.hpp"
#include "ssb_core/kinematic.hpp"
#include "ssb_core/optix_renderer.hpp"
#include "ssb_core/pipeline.hpp"
#include "ssb_core/texture.hpp"

using namespace ssb;

namespace {

Config BaseConfig() { return Config::Load(std::string(SSB_CONFIG_DIR) + "/stage_a.yaml"); }

std::vector<RowJob> Jobs(const Config& c, int n) {
  std::vector<RowJob> jobs(n);
  for (int i = 0; i < n; ++i) {
    jobs[i].record.sequence = i;
    jobs[i].pose.theta = -2.0 + 4.0 * i / n;  // across the whole 240 deg arc
    jobs[i].pose.x = 1.0 + 0.001 * i;
  }
  (void)c;
  return jobs;
}

std::string ReadFile(const std::filesystem::path& p) {
  std::ifstream in(p, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), {}};
}

}  // namespace

TEST(Render, BackendLoadsAndPassesItsSelfCheck) {
  OptixRenderer r(BaseConfig(), DefaultPtxPath(), 16);
  const auto check = r.SelfCheck();
  EXPECT_TRUE(check.at("passed").get<bool>());
  const auto d = r.Describe();
  EXPECT_NE(d.at("optix_library").get<std::string>(), "");
}

TEST(Render, DebugHitsMatchTheHostCylinderAndMountModel) {
  Config c = BaseConfig();
  // Non-nominal mount so every offset term is exercised together (DESIGN.md §12.3).
  c.truth.mount = {0.015, 0.004, -0.01, 0.012, 1e-3, -8e-4, 5e-4};
  OptixRenderer r(c, DefaultPtxPath(), 512);
  const auto jobs = Jobs(c, 512);
  std::vector<uint8_t> pixels;
  std::vector<double> hits;
  r.Render(jobs, pixels, hits);
  const auto columns = DebugColumns(c);
  ASSERT_EQ(hits.size(), jobs.size() * columns.size() * 2);
  double worst = 0;
  size_t differing = 0;
  for (size_t i = 0; i < jobs.size(); ++i) {
    const HeadPose h = TrueHeadPose(c, jobs[i].pose);
    for (int u = 0; u < c.width; ++u) {
      Vec3 d;
      for (int k = 0; k < 3; ++k) d[k] = h.optical[k] + c.PixelTangent(u) * h.line[k];
      const double n = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
      const double oy = h.origin[1], oz = h.origin[2] - c.tunnel_axis_z_m, dy = d[1] / n, dz = d[2] / n;
      const double a = dy * dy + dz * dz, b = 2 * (oy * dy + oz * dz);
      const double cc = oy * oy + oz * oz - c.tunnel_radius_m * c.tunnel_radius_m;
      const double t = (-b + std::sqrt(b * b - 4 * a * cc)) / (2 * a);
      const double x = h.origin[0] + t * d[0] / n, q = c.tunnel_radius_m * std::atan2(oy + t * dy, oz + t * dz);
      differing += pixels[i * c.width + u] != AlbedoCode(WallAlbedo(x, q));
      for (size_t k = 0; k < columns.size(); ++k)
        if (columns[k] == u) {
          worst = std::max(worst, std::abs(hits[(i * columns.size() + k) * 2] - x));
          worst = std::max(worst, std::abs(hits[(i * columns.size() + k) * 2 + 1] - q));
        }
    }
  }
  EXPECT_LT(worst, 2e-6);  // 0.01 px; float ray directions on the GPU
  EXPECT_LT(differing, pixels.size() / 1000) << differing << " of " << pixels.size();
}

TEST(Render, BatchSizeDoesNotChangeAnyByte) {
  const Config c = BaseConfig();
  OptixRenderer r(c, DefaultPtxPath(), 700);
  const auto jobs = Jobs(c, 700);
  std::vector<uint8_t> whole, part, pieces;
  std::vector<double> hits_whole, hits, hit_pieces;
  r.Render(jobs, whole, hits_whole);
  for (size_t first = 0; first < jobs.size(); first += 97) {
    const std::vector<RowJob> slice(jobs.begin() + first, jobs.begin() + std::min(jobs.size(), first + 97));
    r.Render(slice, part, hits);
    pieces.insert(pieces.end(), part.begin(), part.end());
    hit_pieces.insert(hit_pieces.end(), hits.begin(), hits.end());
  }
  EXPECT_EQ(whole, pieces);
  EXPECT_EQ(hits_whole, hit_pieces);
}

TEST(Render, RaysLeavingTheTunnelRejectTheBatch) {
  const Config c = BaseConfig();
  OptixRenderer r(c, DefaultPtxPath(), 4);
  auto jobs = Jobs(c, 4);
  jobs[2].pose.x = c.tunnel_x_max_m;  // half the line lands beyond the wall's end
  std::vector<uint8_t> pixels;
  std::vector<double> hits;
  EXPECT_THROW(r.Render(jobs, pixels, hits), std::runtime_error);
}

// Re-imaging an archived pose stream with another batch size and an artificial
// imaging lag reproduces raw pixels and metadata byte for byte (DESIGN.md §8.2).
TEST(Pipeline, ReimagingIsByteIdenticalAcrossBatchSizeAndLag) {
  Config c = BaseConfig();
  c.profile = {{0, 0}, {0.6, 1}, {2.4, 1}};  // start-up plus about one scan arc
  const auto dir = std::filesystem::temp_directory_path() / ("ssb_test_" + std::to_string(getpid()));
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  const auto first = dir / "first", second = dir / "second";
  nlohmann::json a, b;
  {
    PipelineOptions options{first, {"test"}, "kinematic"};
    options.planned_end_s = c.profile.back()[0];
    Pipeline p(c, std::make_unique<OptixRenderer>(c, DefaultPtxPath(), c.batch_rows), options);
    for (const auto& s : KinematicSource(c).Sample()) p.Push(s);
    p.Finish();
    a = p.Wait();
  }
  const auto poses = first / "evaluation" / "pose_stream.bin";
  std::vector<PoseSample> samples(std::filesystem::file_size(poses) / sizeof(PoseSample));
  std::ifstream(poses, std::ios::binary).read(reinterpret_cast<char*>(samples.data()),
                                              samples.size() * sizeof(PoseSample));
  Config c2 = c;
  c2.batch_rows = 333;
  c2.debug_delay_per_batch_s = 0.002;
  c2.max_queued_batches = 2;  // slow rendering must hold back timing expansion, not grow a queue
  {
    Pipeline p(c2, std::make_unique<OptixRenderer>(c2, DefaultPtxPath(), c2.batch_rows), {second, {"test"}, "file"});
    for (const auto& s : samples) p.Push(s);
    p.Finish();
    b = p.Wait();
  }
  ASSERT_EQ(a.at("status"), "complete");
  ASSERT_EQ(b.at("status"), "complete");
  EXPECT_GT(a.at("rows").get<int64_t>(), 10000);
  EXPECT_EQ(a.at("rows"), b.at("rows"));
  EXPECT_EQ(a.at("motion").at("complete"), true);
  EXPECT_TRUE(b.at("motion").at("complete").is_null());  // archived stream: no plan to judge
  EXPECT_EQ(b.at("performance").at("render_queue_peak_batches"), 2);
  size_t compared = 0;
  for (const auto& sub : {"raw", "metadata", "evaluation"}) {
    for (const auto& e : std::filesystem::directory_iterator(first / sub)) {
      const auto name = e.path().filename();
      if (name == "config_source.yaml" || name == "truth.json") continue;
      EXPECT_EQ(ReadFile(e.path()), ReadFile(second / sub / name)) << sub << "/" << name.string();
      ++compared;
    }
  }
  EXPECT_GE(compared, 12u);
  std::filesystem::remove_all(dir);
}

TEST(Pipeline, TruncatedStreamIsDrainedButReportsMotionIncomplete) {
  Config c = BaseConfig();
  c.profile = {{0, 0}, {0.6, 1}, {1.6, 1}};
  const auto dir = std::filesystem::temp_directory_path() / ("ssb_trunc_" + std::to_string(getpid()));
  std::filesystem::remove_all(dir);
  PipelineOptions options{dir, {"test"}, "kinematic"};
  options.planned_end_s = c.profile.back()[0];
  nlohmann::json summary;
  {
    Pipeline p(c, std::make_unique<OptixRenderer>(c, DefaultPtxPath(), c.batch_rows), options);
    for (const auto& s : KinematicSource(c).Sample())
      if (s.t <= 0.8) p.Push(s);
    p.Finish();
    summary = p.Wait();
  }
  EXPECT_EQ(summary.at("status"), "complete");
  EXPECT_EQ(summary.at("motion").at("complete"), false);
  EXPECT_DOUBLE_EQ(summary.at("motion").at("last_sample_s").get<double>(), 0.8);
  std::filesystem::remove_all(dir);
}

TEST(Render, FullBodyPoseRotatesCameraMountAboutBase) {
  Config c=BaseConfig();c.truth.mount={};c.truth.head_mount_x_m=.2;
  PoseSample p{};p.body_valid=1;p.x=4;p.y=.01;p.z=.3;p.yaw=M_PI/2;
  auto h=TrueHeadPose(c,p);
  EXPECT_NEAR(h.origin[0],4,1e-12);EXPECT_NEAR(h.origin[1],.21,1e-12);
  EXPECT_NEAR(h.origin[2],c.tunnel_axis_z_m,1e-12);
  EXPECT_NEAR(h.line[0],0,1e-12);EXPECT_NEAR(h.line[1],1,1e-12);
  p.yaw=0;p.roll=M_PI/2;h=TrueHeadPose(c,p);
  EXPECT_NEAR(h.origin[1],.01-(c.tunnel_axis_z_m-.3),1e-12);
  EXPECT_NEAR(h.origin[2],.3,1e-12);EXPECT_NEAR(h.optical[1],-1,1e-12);
  p.body_valid=0;h=TrueHeadPose(c,p);
  EXPECT_NEAR(h.origin[0],4.2,1e-12);EXPECT_NEAR(h.origin[1],0,1e-12);
}
