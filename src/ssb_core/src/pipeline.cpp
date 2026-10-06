#include "ssb_core/pipeline.hpp"

#include <atomic>
#include <cmath>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <exception>
#include <fstream>
#include <mutex>
#include <thread>

#include "ssb_core/camera_model.hpp"
#include "ssb_core/session.hpp"
#include "ssb_core/sha256.hpp"
#include "ssb_core/timing.hpp"

namespace ssb {
namespace {

using Clock = std::chrono::steady_clock;

double Seconds(Clock::time_point a, Clock::time_point b) { return std::chrono::duration<double>(b - a).count(); }

struct RenderedBatch {
  int64_t first_sequence = 0;
  size_t rows = 0;
  double last_center = 0;
  std::vector<uint8_t> pixels;
  std::vector<double> hits;
  size_t Bytes() const { return pixels.size() + hits.size() * sizeof(double); }
};

}  // namespace

struct Pipeline::Impl {
  Config config;
  std::unique_ptr<RowRenderer> renderer;
  PipelineOptions options;
  std::filesystem::path root;
  std::vector<int> debug_columns;

  std::mutex mutex;
  std::condition_variable cv;
  std::deque<PoseSample> poses;
  bool input_finished = false;
  std::string producer_error;
  std::deque<std::vector<RowJob>> render_queue;
  size_t render_queue_peak = 0;
  bool timing_done = false;
  std::deque<RenderedBatch> write_queue;
  size_t write_queue_bytes = 0, write_queue_peak = 0;
  bool render_done = false;
  bool failed = false;
  std::exception_ptr error;

  std::atomic<double> latest_pushed{0};
  bool have_first = false;
  double first_pushed = 0;
  Clock::time_point first_push_wall;
  std::atomic<double> latest_written_center{0};
  std::atomic<int64_t> rows_generated{0}, rows_persisted{0};
  // Completed work only: idle heartbeats must not keep a stuck drain alive.
  std::atomic<uint64_t> activity_sequence{0};
  std::atomic<bool> write_done{false}, finalizing{false}, complete{false};
  // Set when Wait ends without a complete session (producer error or commit failure).
  std::atomic<bool> ended_failed{false};
  Clock::time_point start_wall, finish_called_wall;
  double pushed_at_finish = 0, written_at_finish = 0;
  bool finish_called = false;
  nlohmann::json distance_motion;

  nlohmann::json metadata_tables, evaluation_tables, raw_index, timing_stats;
  double render_seconds = 0, write_seconds = 0;
  double render_queue_wait_s = 0, write_queue_wait_s = 0;
  IoStatistics raw_io, table_io, physical_io;
  int64_t batches = 0;
  nlohmann::json progress = nlohmann::json::array();

  std::thread timing_thread, render_thread, write_thread;
  bool waited = false;

  void Fail(std::exception_ptr e) {
    std::lock_guard<std::mutex> lock(mutex);
    if (!failed) error = e;
    failed = true;
    poses.clear();
    render_queue.clear();
    write_queue.clear();
    write_queue_bytes = 0;
    cv.notify_all();
  }

  void TimingLoop();
  void RenderLoop();
  void WriteLoop();
};

Pipeline::Pipeline(const Config& config, std::unique_ptr<RowRenderer> renderer, const PipelineOptions& options)
    : impl_(std::make_unique<Impl>()) {
  auto& s = *impl_;
  config.Validate();
  if (!renderer) throw std::invalid_argument("pipeline needs a renderer");
  s.config = config;
  s.renderer = std::move(renderer);
  s.options = options;
  s.root = options.session;
  s.debug_columns = DebugColumns(config);
  if (std::filesystem::exists(s.root)) throw std::runtime_error("session already exists: " + s.root.string());
  for (const char* d : {"config", "raw", "metadata", "evaluation", "reconstruction", "logs"})
    std::filesystem::create_directories(s.root / d);

  // Reconstruction-visible configuration vs. truth (DESIGN.md §2: T_hat vs T_true).
  WriteJsonAtomic(s.root / "config" / "observable_config.json", config.ObservableJson());
  WriteJsonAtomic(s.root / "evaluation" / "truth.json", config.TruthJson());
  WriteTextAtomic(s.root / "evaluation" / "config_source.yaml", config.source_text);
  nlohmann::json provenance = ProvenanceJson(options.argv);
  provenance["pose_source"] = options.pose_source;
  provenance["inputs"] = options.inputs;
  provenance["config_sha256"] = config.source_sha256;
  WriteJsonAtomic(s.root / "config" / "provenance.json", provenance);
  // Refuse to record anything unless the loaded backend demonstrably works (§12.1).
  try {
    nlohmann::json backend = {{"describe", s.renderer->Describe()}, {"self_check", s.renderer->SelfCheck()}};
    WriteJsonAtomic(s.root / "config" / "backend.json", backend);
    const auto assets=s.renderer->EvaluationAssets();
    if(!assets.is_null()) WriteJsonAtomic(s.root / "evaluation" / "optical_assets.json",assets);
  } catch (const std::exception& e) {
    WriteJsonAtomic(s.root / "session.json", {{"schema", "ssb.session.v1"}, {"status", "failed"},
                                              {"error", std::string("backend self-check: ") + e.what()}});
    throw;
  }
  WriteJsonAtomic(s.root / "session.json", {{"schema", "ssb.session.v1"}, {"status", "running"},
                                            {"pose_source", options.pose_source}});
  s.start_wall = Clock::now();
  s.timing_thread = std::thread([&s] { s.TimingLoop(); });
  s.render_thread = std::thread([&s] { s.RenderLoop(); });
  s.write_thread = std::thread([&s] { s.WriteLoop(); });
}

Pipeline::~Pipeline() {
  if (!impl_->waited) {
    try {
      Finish();
      Wait();
    } catch (...) {
    }
  }
}

nlohmann::json FileIdentity(const std::filesystem::path& path) {
  return {{"path", std::filesystem::absolute(path).string()}, {"sha256", Sha256File(path)}};
}

void Pipeline::Push(const PoseSample& sample) {
  auto& s = *impl_;
  std::lock_guard<std::mutex> lock(s.mutex);
  if (s.failed) return; // worker error is reported by Wait; never accumulate more input
  if (s.input_finished) throw std::logic_error("pose pushed after Finish");
  if (!s.have_first) s.have_first = true, s.first_pushed = sample.t, s.first_push_wall = Clock::now();
  s.poses.push_back(sample);
  s.latest_pushed = sample.t;
  s.cv.notify_all();
}

void Pipeline::FinishDistanceMotion(double distance,double speed) {
  auto& s=*impl_;
  const auto& d=s.config.distance_stop;
  if(!d.Enabled() || !std::isfinite(distance) || !std::isfinite(speed) ||
     std::abs(distance-d.target_m)>d.tolerance_m || std::abs(speed)>d.speed_tolerance_m_s)
    throw std::invalid_argument("distance motion cannot complete before target and parking");
  {
    std::lock_guard<std::mutex> lock(s.mutex);
    if(s.input_finished) throw std::logic_error("distance motion reported after Finish");
    s.distance_motion={{"target_estimated_m",d.target_m},{"estimated_distance_m",distance},
      {"estimated_speed_m_s",speed},{"completion_basis","dual_encoder_distance_and_park"}};
  }
  Finish();
}

void Pipeline::Finish(const std::string& producer_error) {
  auto& s = *impl_;
  std::lock_guard<std::mutex> lock(s.mutex);
  if (!producer_error.empty() && s.producer_error.empty()) s.producer_error = producer_error;
  if (s.input_finished) return;
  s.input_finished = true;
  s.finish_called = true;
  s.finish_called_wall = Clock::now();
  s.pushed_at_finish = s.latest_pushed;
  s.written_at_finish = s.latest_written_center;
  s.cv.notify_all();
}

nlohmann::json Pipeline::Progress() const {
  auto& s = *impl_;
  std::lock_guard<std::mutex> lock(s.mutex);
  return {{"rows_generated", s.rows_generated.load()},
          {"rows_saved", s.rows_persisted.load()},
          {"sim_time_pushed", s.latest_pushed.load()},
          {"sim_time_written", s.latest_written_center.load()},
          {"activity_sequence", s.activity_sequence.load()},
          {"phase", s.complete ? "complete" : s.ended_failed || s.failed ? "failed" :
                    s.finalizing ? "finalizing" :
                    s.write_done ? "joining" : s.render_done ? "syncing" :
                    s.input_finished ? "draining" : "capturing"},
          {"failed", s.failed || !s.producer_error.empty()}};
}

void Pipeline::Impl::TimingLoop() {
  try {
    const auto dir = root / "metadata", eval = root / "evaluation";
    const auto activity = [this] { ++activity_sequence; };
    TableWriter pose_table(eval / "pose_stream.bin", DtypeJson(PoseSampleFields()), sizeof(PoseSample), activity);
    TableWriter truth_table(eval / "row_truth.bin", DtypeJson(RowTruthFields()), sizeof(RowTruthRecord), activity);
    TableWriter rows_table(dir / "rows.bin", DtypeJson(RowFields()), sizeof(RowRecord), activity);
    TableWriter scan_table(dir / "scan_edges.bin", DtypeJson(EdgeFields()), sizeof(EdgeRecord), activity);
    TableWriter odo_table(dir / "odometer_edges.bin", DtypeJson(EdgeFields()), sizeof(EdgeRecord), activity);
    TableWriter right_odo_table(dir / "odometer_right_edges.bin", DtypeJson(EdgeFields()), sizeof(EdgeRecord), activity);
    TableWriter gate_table(dir / "gate_events.bin", DtypeJson(GateFields()), sizeof(GateRecord), activity);
    TableWriter drop_table(dir / "dropped_rows.bin", DtypeJson(DroppedRowFields()), sizeof(DroppedRowRecord), activity);
    TimingEngine engine(config);
    TimingOutput out;
    std::vector<RowJob> batch;
    std::vector<RowRecord> row_records;
    std::vector<RowTruthRecord> truth_records;
    auto drain = [&]() -> bool {
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (failed) return false;
      }
      scan_table.Append(out.scan_edges);
      odo_table.Append(out.odo_edges);right_odo_table.Append(out.right_odo_edges);
      gate_table.Append(out.gates);
      drop_table.Append(out.dropped);
      row_records.clear();
      truth_records.clear();
      for (const auto& job : out.rows) {
        ++rows_generated;
        row_records.push_back(job.record);
        const auto& p = job.pose;
        truth_records.push_back({job.record.sequence, job.record.t_center, p.theta, p.omega, p.x, p.v,p.y,p.z,p.roll,p.pitch,p.yaw,p.body_valid});
        batch.push_back(job);
        if (batch.size() == static_cast<size_t>(config.batch_rows)) {
          std::unique_lock<std::mutex> lock(mutex);
          const auto waiting=Clock::now();
          cv.wait(lock, [&] {
            return failed || render_queue.size() < static_cast<size_t>(config.max_queued_batches);
          });
          render_queue_wait_s+=Seconds(waiting,Clock::now());
          if (failed) return false;
          render_queue.push_back(std::move(batch));
          render_queue_peak = std::max(render_queue_peak, render_queue.size());
          batch.clear();
          cv.notify_all();
        }
      }
      rows_table.Append(row_records);
      truth_table.Append(truth_records);
      out.Clear();
      return true;
    };
    for (;;) {
      std::deque<PoseSample> local;
      bool finished;
      {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&] { return failed || input_finished || !poses.empty(); });
        if (failed) return;
        local.swap(poses);
        finished = input_finished;
      }
      for (const auto& p : local) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          if (failed) return;
        }
        pose_table.Append(&p, 1);
        engine.Push(p, out);
        if (!drain()) return;
        ++activity_sequence;
      }
      if (finished) {
        std::lock_guard<std::mutex> lock(mutex);
        if (!poses.empty()) continue;
        break;
      }
    }
    engine.Finish(out);
    if (!drain()) return;
    nlohmann::json meta = {{"rows", rows_table.Close()}, {"scan_edges", scan_table.Close()},
                           {"odometer_edges", odo_table.Close()}, {"odometer_right_edges",right_odo_table.Close()}, {"gate_events", gate_table.Close()},
                           {"dropped_rows", drop_table.Close()}};
    nlohmann::json evalj = {{"pose_stream", pose_table.Close()}, {"row_truth", truth_table.Close()}};
    const auto& st = engine.Stats();
    std::lock_guard<std::mutex> lock(mutex);
    if (failed) return;
    if (!batch.empty()) render_queue.push_back(std::move(batch));
    for(const auto* table:{&rows_table,&scan_table,&odo_table,&right_odo_table,&gate_table,
                          &drop_table,&pose_table,&truth_table}) table_io.Merge(table->Statistics());
    metadata_tables = meta;
    evaluation_tables = evalj;
    timing_stats = {{"samples", st.samples}, {"scan_edges", st.scan_edges}, {"odometer_edges", st.odo_edges},
                    {"odometer_right_edges", st.right_odo_edges},
                    {"gate_events", st.gate_events}, {"triggers", st.triggers},
                    {"triggers_outside_gate", st.triggers_outside_gate}, {"rows", st.rows},
                    {"dropped_early_edge", st.dropped_early_edge}, {"dropped_no_period", st.dropped_no_period},
                    {"dropped_reverse", st.dropped_reverse}, {"dropped_overrun", st.dropped_overrun},
                    {"dropped_stream_end", st.dropped_stream_end}};
    timing_done = true;
    ++activity_sequence;
    cv.notify_all();
  } catch (...) {
    Fail(std::current_exception());
  }
}

void Pipeline::Impl::RenderLoop() {
  try {
    for (;;) {
      std::vector<RowJob> jobs;
      {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&] { return failed || timing_done || !render_queue.empty(); });
        if (failed) return;
        if (render_queue.empty()) break;  // timing done and drained
        jobs = std::move(render_queue.front());
        render_queue.pop_front();
        cv.notify_all();
      }
      RenderedBatch b;
      b.first_sequence = jobs.front().record.sequence;
      b.rows = jobs.size();
      b.last_center = jobs.back().record.t_center;
      const auto t0 = Clock::now();
      renderer->Render(jobs, b.pixels, b.hits);
      ++activity_sequence;
      const double dt = Seconds(t0, Clock::now());
      if (config.debug_delay_per_batch_s > 0)
        std::this_thread::sleep_for(std::chrono::duration<double>(config.debug_delay_per_batch_s));
      std::unique_lock<std::mutex> lock(mutex);
      render_seconds += dt;
      const size_t bytes = b.Bytes();
      const auto waiting=Clock::now();
      cv.wait(lock, [&] {
        return failed || write_queue.empty() || write_queue_bytes + bytes <= config.write_queue_bytes;
      });
      write_queue_wait_s+=Seconds(waiting,Clock::now());
      if (failed) return;
      write_queue_bytes += bytes;
      write_queue_peak = std::max(write_queue_peak, write_queue_bytes);
      write_queue.push_back(std::move(b));
      cv.notify_all();
    }
    std::lock_guard<std::mutex> lock(mutex);
    render_done = true;
    ++activity_sequence;
    cv.notify_all();
  } catch (...) {
    Fail(std::current_exception());
  }
}

void Pipeline::Impl::WriteLoop() {
  try {
    const auto activity = [this] { ++activity_sequence; };
    BlockWriter blocks(root / "raw", config.width, config.block_rows, activity);
    const size_t columns = debug_columns.size();
    nlohmann::json dtype = nlohmann::json::array({{"sequence", "<i8"}});
    if (columns) dtype.push_back({"hits", "<f8", {columns, 2}});
    TableWriter hit_table(root / "evaluation" / "debug_hits.bin", dtype, 8 + columns * 16, activity);
    std::vector<unsigned char> record(8 + columns * 16);
    for (;;) {
      RenderedBatch b;
      {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&] { return failed || render_done || !write_queue.empty(); });
        if (failed) return;
        if (write_queue.empty()) break;
        b = std::move(write_queue.front());
        write_queue.pop_front();
        write_queue_bytes -= b.Bytes();
        cv.notify_all();
      }
      const auto t0 = Clock::now();
      blocks.Append(b.pixels.data(), b.rows, b.first_sequence);
      rows_persisted = blocks.RowsPersisted();
      record.resize(b.rows*(8+columns*16));
      for (size_t i = 0; i < b.rows; ++i) {
        auto* target=record.data()+i*(8+columns*16);
        const int64_t seq = b.first_sequence + static_cast<int64_t>(i);
        std::memcpy(target, &seq, 8);
        if (columns) std::memcpy(target + 8, b.hits.data() + i * columns * 2, columns * 16);
      }
      hit_table.Append(record.data(), b.rows);
      const auto t1 = Clock::now();
      latest_written_center = b.last_center;
      ++activity_sequence;
      std::lock_guard<std::mutex> lock(mutex);
      write_seconds += Seconds(t0, t1);
      ++batches;
      progress.push_back({{"wall_s", Seconds(start_wall, t1)}, {"rows_written", blocks.RowsWritten()},
                          {"sim_time_written", b.last_center}, {"sim_time_pushed", latest_pushed.load()}});
    }
    nlohmann::json raw = blocks.Close();
    rows_persisted = blocks.RowsPersisted();
    nlohmann::json hits = hit_table.Close();
    hits["columns"] = debug_columns;
    std::lock_guard<std::mutex> lock(mutex);
    raw_io=blocks.Statistics();
    table_io.Merge(hit_table.Statistics());
    raw_index = raw;
    evaluation_tables["debug_hits"] = hits;
    write_done = true;
    ++activity_sequence;
  } catch (...) {
    Fail(std::current_exception());
  }
}

nlohmann::json Pipeline::Wait() {
  auto& s = *impl_;
  if (s.waited) throw std::logic_error("Pipeline::Wait called twice");
  Finish();
  s.waited = true;
  s.timing_thread.join();
  s.render_thread.join();
  s.write_thread.join();
  if (s.failed) {
    std::string what = "unknown error";
    try {
      std::rethrow_exception(s.error);
    } catch (const std::exception& e) {
      what = e.what();
    } catch (...) {
    }
    WriteJsonAtomic(s.root / "session.json", {{"schema", "ssb.session.v1"}, {"status", "failed"},
                                              {"error", what}, {"pose_source", s.options.pose_source}});
    std::rethrow_exception(s.error);
  }
  s.finalizing = true;
  ++s.activity_sequence;
  try {
    const int64_t rows = s.raw_index.at("rows").get<int64_t>();
    const double sim_first = s.first_pushed;
    WriteJsonAtomic(s.root / "metadata" / "manifest.json", s.metadata_tables);
    WriteJsonAtomic(s.root / "evaluation" / "manifest.json", s.evaluation_tables);
    WriteJsonAtomic(s.root / "raw" / "index.json", s.raw_index);
    ++s.activity_sequence;
    // "complete" means every sample that arrived is imaged and on disk; whether the
    // planned motion was actually run is reported separately.
    const double planned = s.options.planned_end_s;
    const double tolerance = 0.5 * s.config.sample_period_s;
    nlohmann::json motion = {{"first_sample_s", s.have_first ? nlohmann::json(sim_first) : nlohmann::json()},
                             {"last_sample_s", s.latest_pushed.load()}};
    if(s.config.distance_stop.Enabled() && s.options.pose_source=="gazebo_contact") {
      motion["complete"]=!s.distance_motion.is_null();
      motion["planned_end_s"]=s.latest_pushed.load(); // permits exact archived replay
      motion["completion_basis"]="dual_encoder_distance_and_park";
      if(!s.distance_motion.is_null()) motion.update(s.distance_motion);
    } else if (std::isfinite(planned)) {
      motion["planned_end_s"] = planned;
      motion["complete"] = s.latest_pushed.load() >= planned - tolerance;
    } else {
      motion["planned_end_s"] = nullptr;
      motion["complete"] = nullptr;
    }
    // Content identity of the descriptive files at completion; tables and blocks carry
    // their own hashes in the manifests listed here.
    nlohmann::json files;
    for (const char* name : {"config/observable_config.json", "config/provenance.json", "config/backend.json",
                             "evaluation/truth.json", "evaluation/config_source.yaml", "evaluation/manifest.json",
                             "metadata/manifest.json", "raw/index.json"})
      files[name] = Sha256File(s.root / name);
    if(std::filesystem::exists(s.root / "evaluation" / "optical_assets.json"))
      files["evaluation/optical_assets.json"]=Sha256File(s.root / "evaluation" / "optical_assets.json");
    // Protect the complete archived physical input set, including SDF, spec, config and images.
    const auto physical = s.root / "evaluation" / "physical";
    if (std::filesystem::exists(physical)) {
      std::vector<std::filesystem::path> directories{physical};
      for (const auto& entry : std::filesystem::recursive_directory_iterator(physical)) {
        if (entry.is_directory()) directories.push_back(entry.path());
        if (!entry.is_regular_file()) continue;
        files[entry.path().lexically_relative(s.root).generic_string()] = Sha256File(entry.path());
        const auto begin=Clock::now();SyncFile(entry.path());
        ++s.activity_sequence;
        const double seconds=Seconds(begin,Clock::now());
        s.physical_io.sync_seconds+=seconds;
        s.physical_io.longest_sync_s=std::max(s.physical_io.longest_sync_s,seconds);
        ++s.physical_io.files;s.physical_io.bytes+=entry.file_size();
      }
      for(auto directory=directories.rbegin();directory!=directories.rend();++directory) {
        SyncDirectory(*directory);
        ++s.activity_sequence;
      }
    }
    // Persist directory entries for initial files, snapshots and the session itself.
    for(const char* name:{"metadata","evaluation","config","logs"}) {
      SyncDirectory(s.root/name);
      ++s.activity_sequence;
    }
    SyncDirectory(s.root);
    SyncDirectory(s.root.parent_path().empty() ? "." : s.root.parent_path());
    const auto end=Clock::now();
    const double wall = Seconds(s.start_wall, end);
    const double imaging_wall = s.have_first ? Seconds(s.first_push_wall, end) : 0;
    const double producer_wall = s.have_first && s.finish_called ? Seconds(s.first_push_wall, s.finish_called_wall) : 0;
    double sim_last = sim_first;
    if (!s.progress.empty()) sim_last = s.progress.back().at("sim_time_written").get<double>();
    const nlohmann::json performance = {
        {"wall_seconds", wall},
        {"rows", rows},
        {"average_rows_per_wall_second", wall > 0 ? rows / wall : 0},
        {"render_seconds", s.render_seconds},
        {"render_rows_per_second", s.render_seconds > 0 ? rows / s.render_seconds : 0},
        {"write_seconds", s.write_seconds},
        {"io", {{"raw",s.raw_io.Json()},{"tables",s.table_io.Json()},
                 {"physical_snapshot",s.physical_io.Json()},
                 {"render_queue_wait_s",s.render_queue_wait_s},{"write_queue_wait_s",s.write_queue_wait_s}}},
        // DESIGN.md §11, both measured from the first pose sample (excludes start-up):
        // dynamics = pose stream span per wall second while it was produced; imaging =
        // the same span per wall second until the last row was on disk.
        {"dynamics_rtf", producer_wall > 0 ? (s.pushed_at_finish - sim_first) / producer_wall : 0},
        {"imaging_progress_rtf", imaging_wall > 0 ? (s.latest_pushed.load() - sim_first) / imaging_wall : 0},
        {"last_row_center", sim_last},
        {"sim_time_pushed_at_finish", s.pushed_at_finish},
        {"sim_time_written_at_finish", s.written_at_finish},
        {"wall_seconds_after_finish", s.finish_called ? Seconds(s.finish_called_wall, end) : 0},
        {"write_queue_peak_bytes", s.write_queue_peak},
        {"render_queue_peak_batches", s.render_queue_peak},
        {"render_queue_limit_batches", s.config.max_queued_batches},
        {"batches", s.batches},
        {"batch_rows", s.config.batch_rows},
        {"backend_final", s.renderer->Describe()},
        {"note", "wall_seconds starts after the backend self-check; rates include payload/index durability and end before the final completion marker"}};
    WriteJsonAtomic(s.root / "logs" / "performance.json", {{"summary", performance}, {"progress", s.progress}});
    nlohmann::json summary = {{"schema", "ssb.session.v1"},
                                    {"status", s.producer_error.empty() ? "complete" : "failed"},
                                    {"files", files},
                                    {"motion", motion},
                                    {"pose_source", s.options.pose_source},
                                    {"rows", rows},
                                    {"timing", s.timing_stats},
                                    {"performance", performance}};
    if (!s.producer_error.empty()) {
      summary["error"] = s.producer_error;
      summary["motion"]["complete"] = false;
    }
    WriteJsonAtomic(s.root / "session.json", summary);
    if (!s.producer_error.empty()) {
      s.ended_failed = true;
      ++s.activity_sequence;
      throw std::runtime_error(s.producer_error);
    }
    s.complete = true;
    ++s.activity_sequence;
    return summary;
  } catch(const std::exception& error) {
    s.ended_failed = true;
    ++s.activity_sequence;
    // Commit failures also leave an explicitly failed session when the filesystem
    // still permits it. Preserve any already-written recovery indices and counts.
    try {
      nlohmann::json failed;
      std::ifstream(s.root/"session.json")>>failed;
      failed["status"]="failed";failed["error"]=error.what();
      WriteJsonAtomic(s.root/"session.json",failed);
    } catch(...) { /* The original persistence error remains authoritative. */ }
    throw;
  }
}

}  // namespace ssb
