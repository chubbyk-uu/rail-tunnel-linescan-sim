#include <gtest/gtest.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <cerrno>
#include <atomic>
#include <mutex>
#include "ssb_core/session.hpp"
#include <chrono>
#include <fstream>
#include "ssb_core/kinematic.hpp"
#include "ssb_core/pipeline.hpp"
#include "ssb_core/sha256.hpp"

namespace {
std::atomic<bool> reject_block_sync{false};
std::mutex sync_mutex;
std::vector<std::string> sync_trace;
}
// Fault injection is linked into this test executable only, never production.
extern "C" int fsync(int fd) {
  char name[4096];const auto n=readlink(("/proc/self/fd/"+std::to_string(fd)).c_str(),name,sizeof(name)-1);
  const std::string path=n>=0 ? std::string(name,size_t(n)) : "";
  if(reject_block_sync && path.find("/raw/block_")!=std::string::npos) {errno=EIO;return -1;}
  const int result=int(syscall(SYS_fsync,fd));
  if(result==0) {std::lock_guard<std::mutex> lock(sync_mutex);sync_trace.push_back(path);}
  return result;
}

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

TEST(PipelineCompletion, HashesEveryArchivedPhysicalFile) {
  auto c=ssb::Config::Load(std::string(SSB_CONFIG_DIR)+"/stage_a.yaml");
  c.profile={{0,1},{.002,1}};c.batch_rows=4096;
  const auto root=std::filesystem::temp_directory_path()/("ssb_physical_hash_"+std::to_string(getpid()));
  std::filesystem::remove_all(root);
  ssb::Pipeline p(c,std::make_unique<FailingRenderer>(c.width),{root,{"test"},"kinematic"});
  const auto physical=root/"evaluation/physical";
  std::filesystem::create_directories(physical/"nested");
  for(const char* name:{"world.sdf","spec.yaml","capture.yaml","physical_manifest.json","nested/rail.png"})
    std::ofstream(physical/name)<<name;
  for(const auto& sample:ssb::KinematicSource(c).Sample())p.Push(sample);
  p.Finish();const auto summary=p.Wait();
  for(const auto& entry:std::filesystem::recursive_directory_iterator(physical)) {
    if(!entry.is_regular_file())continue;
    const auto name=entry.path().lexically_relative(root).generic_string();
    EXPECT_EQ(summary.at("files").at(name),ssb::Sha256File(entry.path()));
  }
  const auto original=summary.at("files").at("evaluation/physical/world.sdf");
  std::ofstream(physical/"world.sdf")<<"changed";
  EXPECT_NE(original,ssb::Sha256File(physical/"world.sdf"));
  std::filesystem::remove_all(root);
}

TEST(PipelineFailure, ProducerFailureDrainsAcceptedInputAndKeepsTailIndex) {
  auto c=ssb::Config::Load(std::string(SSB_CONFIG_DIR)+"/stage_a.yaml");
  c.profile={{0,1},{.002,1}};c.start_theta_rad=0;c.batch_rows=4096;c.debug_column_stride=0;
  const auto root=std::filesystem::temp_directory_path()/("ssb_producer_failure_"+std::to_string(getpid()));
  std::filesystem::remove_all(root);
  ssb::Pipeline p(c,std::make_unique<FailingRenderer>(c.width),{root,{"test"},"kinematic"});
  const auto samples=ssb::KinematicSource(c).Sample();
  for(const auto& sample:samples)p.Push(sample);
  p.Finish("injected producer fault");
  EXPECT_TRUE(p.Progress().at("failed"));
  EXPECT_THROW(p.Wait(),std::runtime_error);
  nlohmann::json summary,index;std::ifstream(root/"session.json")>>summary;
  std::ifstream(root/"raw/index.json")>>index;
  EXPECT_EQ(summary.at("status"),"failed");
  EXPECT_FALSE(summary.at("motion").at("complete"));
  ASSERT_GT(summary.at("rows").get<int64_t>(),0);
  EXPECT_EQ(summary.at("rows"),index.at("rows"));
  EXPECT_LT(index.at("blocks").back().at("rows").get<int>(),c.block_rows);
  EXPECT_EQ(std::filesystem::file_size(root/"evaluation/pose_stream.bin"),samples.size()*sizeof(ssb::PoseSample));
  for(const auto& block:index.at("blocks"))
    EXPECT_EQ(ssb::Sha256File(root/"raw"/block.at("file").get<std::string>()),block.at("sha256"));
  std::filesystem::remove_all(root);
}

TEST(PipelineCompletion, WaitFinishesInputWithoutExplicitFinish) {
  auto c=ssb::Config::Load(std::string(SSB_CONFIG_DIR)+"/stage_a.yaml");
  c.profile={{0,1},{.002,1}};c.start_theta_rad=0;c.batch_rows=4096;c.debug_column_stride=0;
  const auto root=std::filesystem::temp_directory_path()/("ssb_wait_"+std::to_string(getpid()));
  std::filesystem::remove_all(root);
  ssb::Pipeline p(c,std::make_unique<FailingRenderer>(c.width),{root,{"test"},"kinematic"});
  const auto samples=ssb::KinematicSource(c).Sample();
  for(const auto& sample:samples)p.Push(sample);
  const auto summary=p.Wait();
  EXPECT_EQ(summary.at("status"),"complete");
  EXPECT_GT(summary.at("rows").get<int64_t>(),0);
  EXPECT_THROW(p.Push(samples.back()),std::logic_error);
  p.Finish();p.Finish();
  EXPECT_THROW(p.Wait(),std::logic_error);
  std::filesystem::remove_all(root);
}

TEST(Persistence, SyncFailureCannotProduceACompleteSession) {
  auto c=ssb::Config::Load(std::string(SSB_CONFIG_DIR)+"/stage_a.yaml");
  c.profile={{0,1},{.002,1}};c.start_theta_rad=0;c.batch_rows=4096;c.debug_column_stride=0;
  const auto root=std::filesystem::temp_directory_path()/("ssb_sync_failure_"+std::to_string(getpid()));
  std::filesystem::remove_all(root);
  ssb::Pipeline p(c,std::make_unique<FailingRenderer>(c.width),{root,{"test"},"kinematic"});
  for(const auto& sample:ssb::KinematicSource(c).Sample())p.Push(sample);
  reject_block_sync=true;
  EXPECT_THROW(p.Wait(),std::system_error);
  reject_block_sync=false;
  nlohmann::json summary;std::ifstream(root/"session.json")>>summary;
  EXPECT_EQ(summary.at("status"),"failed");
  EXPECT_NE(summary.at("error").get<std::string>().find("fsync"),std::string::npos);
  EXPECT_FALSE(std::filesystem::exists(root/"raw/index.json"));
  EXPECT_TRUE(std::filesystem::exists(root/"raw/block_000000.u8"));
  std::filesystem::remove_all(root);
}

TEST(Persistence, PayloadsAndTheirDirectoriesPrecedeTheCompleteMarker) {
  auto c=ssb::Config::Load(std::string(SSB_CONFIG_DIR)+"/stage_a.yaml");
  c.profile={{0,1},{.002,1}};c.start_theta_rad=0;c.batch_rows=4096;c.debug_column_stride=0;
  const auto root=std::filesystem::temp_directory_path()/("ssb_sync_order_"+std::to_string(getpid()));
  std::filesystem::remove_all(root);
  {std::lock_guard<std::mutex> lock(sync_mutex);sync_trace.clear();}
  ssb::Pipeline p(c,std::make_unique<FailingRenderer>(c.width),{root,{"test"},"kinematic"});
  for(const auto& sample:ssb::KinematicSource(c).Sample())p.Push(sample);
  const auto summary=p.Wait();ASSERT_EQ(summary.at("status"),"complete");
  std::lock_guard<std::mutex> lock(sync_mutex);
  auto marker=std::find(sync_trace.rbegin(),sync_trace.rend(),(root/"session.json.tmp").string());
  ASSERT_NE(marker,sync_trace.rend());
  auto complete=marker.base()-1;
  for(const auto& entry:std::filesystem::recursive_directory_iterator(root)) {
    if(!entry.is_regular_file() || entry.path().filename()=="session.json")continue;
    // Atomic JSON files are synced before rename under their .tmp names.
    auto name=entry.path().string();
    if(entry.path().extension()==".json" || entry.path().filename()=="config_source.yaml")name+=".tmp";
    EXPECT_NE(std::find(sync_trace.begin(),complete,name),complete)<<name;
  }
  for(const char* name:{"raw","metadata","evaluation","config","logs"})
    EXPECT_NE(std::find(sync_trace.begin(),complete,(root/name).string()),complete)<<name;
  EXPECT_GT(summary.at("performance").at("io").at("raw").at("files").get<size_t>(),0u);
  EXPECT_GT(summary.at("performance").at("io").at("raw").at("bytes").get<size_t>(),0u);
  std::filesystem::remove_all(root);
}
