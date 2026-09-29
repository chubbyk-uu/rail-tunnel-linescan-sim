// Offline pose stream -> session. Sources: the config's kinematic profile, or an
// archived pose stream (re-imaging, DESIGN.md §8.2).
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "ssb_core/kinematic.hpp"
#include "ssb_core/optix_renderer.hpp"
#include "ssb_core/pipeline.hpp"
#include "ssb_core/sha256.hpp"

namespace {

void Usage() {
  std::cerr << "usage: ssb_render --config FILE --session DIR [--poses POSE_STREAM.bin]\n"
               "                  [--batch-rows N] [--debug-delay S] [--realtime] [--ptx FILE]\n";
}

std::vector<ssb::PoseSample> ReadPoses(const std::filesystem::path& path) {
  // A session's pose stream: verify it against the manifest beside it when present.
  const auto manifest = path.parent_path() / "manifest.json";
  if (std::filesystem::exists(manifest)) {
    std::ifstream m(manifest);
    const auto j = nlohmann::json::parse(m);
    const auto& entry = j.at("pose_stream");
    if (entry.at("file") != path.filename().string() || entry.at("sha256") != ssb::Sha256File(path))
      throw std::runtime_error("pose stream does not match its manifest: " + path.string());
  }
  const auto bytes = std::filesystem::file_size(path);
  if (bytes == 0 || bytes % sizeof(ssb::PoseSample)) throw std::runtime_error("bad pose stream size");
  std::vector<ssb::PoseSample> samples(bytes / sizeof(ssb::PoseSample));
  std::ifstream in(path, std::ios::binary);
  in.read(reinterpret_cast<char*>(samples.data()), static_cast<std::streamsize>(bytes));
  if (!in) throw std::runtime_error("cannot read " + path.string());
  return samples;
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<std::string> args(argv, argv + argc);
  std::string config_path, session, poses, ptx = ssb::DefaultPtxPath().string();
  int batch_rows = 0;
  double debug_delay = -1;
  bool realtime = false;
  for (size_t i = 1; i < args.size(); ++i) {
    auto next = [&]() -> std::string {
      if (i + 1 >= args.size()) throw std::runtime_error("missing value for " + args[i]);
      return args[++i];
    };
    if (args[i] == "--config") config_path = next();
    else if (args[i] == "--session") session = next();
    else if (args[i] == "--poses") poses = next();
    else if (args[i] == "--batch-rows") batch_rows = std::stoi(next());
    else if (args[i] == "--debug-delay") debug_delay = std::stod(next());
    else if (args[i] == "--realtime") realtime = true;
    else if (args[i] == "--ptx") ptx = next();
    else {
      Usage();
      return 2;
    }
  }
  if (config_path.empty() || session.empty()) {
    Usage();
    return 2;
  }
  try {
    ssb::Config config = ssb::Config::Load(config_path);
    if (batch_rows > 0) config.batch_rows = batch_rows;
    if (debug_delay >= 0) config.debug_delay_per_batch_s = debug_delay;
    config.Validate();
    const std::vector<ssb::PoseSample> samples =
        poses.empty() ? ssb::KinematicSource(config).Sample() : ReadPoses(poses);
    auto renderer = std::make_unique<ssb::OptixRenderer>(config, ptx, static_cast<size_t>(config.batch_rows));
    ssb::Pipeline pipeline(config, std::move(renderer),
                           {session, args, poses.empty() ? "kinematic" : "file:" + poses});
    const auto start = std::chrono::steady_clock::now();
    for (const auto& s : samples) {
      // --realtime paces the producer like a real-time simulator, to exercise lag.
      if (realtime) std::this_thread::sleep_until(start + std::chrono::duration<double>(s.t - samples.front().t));
      pipeline.Push(s);
    }
    pipeline.Finish();
    const auto summary = pipeline.Wait();
    std::cout << summary.dump(2) << std::endl;
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "ssb_render: " << e.what() << std::endl;
    return 1;
  }
}
