#include "ssb_core/pipeline.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <exception>
#include <mutex>
#include <thread>

#include "ssb_core/camera_model.hpp"
#include "ssb_core/session.hpp"
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
  std::deque<std::vector<RowJob>> render_queue;
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
  Clock::time_point start_wall, finish_called_wall;
  double pushed_at_finish = 0, written_at_finish = 0;
  bool finish_called = false;

  nlohmann::json metadata_tables, evaluation_tables, raw_index, timing_stats;
  double render_seconds = 0, write_seconds = 0;
  int64_t batches = 0;
  nlohmann::json progress = nlohmann::json::array();

  std::thread timing_thread, render_thread, write_thread;
  bool waited = false;

  void Fail(std::exception_ptr e) {
    std::lock_guard<std::mutex> lock(mutex);
    if (!failed) error = e;
    failed = true;
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
  WriteJsonAtomic(s.root / "config" / "provenance.json", ProvenanceJson(options.argv));
  // Refuse to record anything unless the loaded backend demonstrably works (§12.1).
  try {
    nlohmann::json backend = {{"describe", s.renderer->Describe()}, {"self_check", s.renderer->SelfCheck()}};
    WriteJsonAtomic(s.root / "config" / "backend.json", backend);
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

void Pipeline::Push(const PoseSample& sample) {
  auto& s = *impl_;
  std::lock_guard<std::mutex> lock(s.mutex);
  if (s.input_finished) throw std::logic_error("pose pushed after Finish");
  if (!s.have_first) s.have_first = true, s.first_pushed = sample.t, s.first_push_wall = Clock::now();
  s.poses.push_back(sample);
  s.latest_pushed = sample.t;
  s.cv.notify_all();
}

void Pipeline::Finish() {
  auto& s = *impl_;
  std::lock_guard<std::mutex> lock(s.mutex);
  if (s.input_finished) return;
  s.input_finished = true;
  s.finish_called = true;
  s.finish_called_wall = Clock::now();
  s.pushed_at_finish = s.latest_pushed;
  s.written_at_finish = s.latest_written_center;
  s.cv.notify_all();
}

void Pipeline::Impl::TimingLoop() {
  try {
    const auto dir = root / "metadata", eval = root / "evaluation";
    TableWriter pose_table(eval / "pose_stream.bin", DtypeJson(PoseSampleFields()), sizeof(PoseSample));
    TableWriter truth_table(eval / "row_truth.bin", DtypeJson(RowTruthFields()), sizeof(RowTruthRecord));
    TableWriter rows_table(dir / "rows.bin", DtypeJson(RowFields()), sizeof(RowRecord));
    TableWriter scan_table(dir / "scan_edges.bin", DtypeJson(EdgeFields()), sizeof(EdgeRecord));
    TableWriter odo_table(dir / "odometer_edges.bin", DtypeJson(EdgeFields()), sizeof(EdgeRecord));
    TableWriter gate_table(dir / "gate_events.bin", DtypeJson(GateFields()), sizeof(GateRecord));
    TableWriter drop_table(dir / "dropped_rows.bin", DtypeJson(DroppedRowFields()), sizeof(DroppedRowRecord));
    TimingEngine engine(config);
    TimingOutput out;
    std::vector<RowJob> batch;
    std::vector<RowRecord> row_records;
    std::vector<RowTruthRecord> truth_records;
    auto drain = [&] {
      scan_table.Append(out.scan_edges);
      odo_table.Append(out.odo_edges);
      gate_table.Append(out.gates);
      drop_table.Append(out.dropped);
      row_records.clear();
      truth_records.clear();
      for (const auto& job : out.rows) {
        row_records.push_back(job.record);
        const auto& p = job.pose;
        truth_records.push_back({job.record.sequence, job.record.t_center, p.theta, p.omega, p.x, p.v});
        batch.push_back(job);
        if (batch.size() == static_cast<size_t>(config.batch_rows)) {
          std::lock_guard<std::mutex> lock(mutex);
          render_queue.push_back(std::move(batch));
          batch.clear();
          cv.notify_all();
        }
      }
      rows_table.Append(row_records);
      truth_table.Append(truth_records);
      out.Clear();
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
        pose_table.Append(&p, 1);
        engine.Push(p, out);
        drain();
      }
      if (finished) {
        std::lock_guard<std::mutex> lock(mutex);
        if (!poses.empty()) continue;
        break;
      }
    }
    engine.Finish(out);
    drain();
    nlohmann::json meta = {{"rows", rows_table.Close()}, {"scan_edges", scan_table.Close()},
                           {"odometer_edges", odo_table.Close()}, {"gate_events", gate_table.Close()},
                           {"dropped_rows", drop_table.Close()}};
    nlohmann::json evalj = {{"pose_stream", pose_table.Close()}, {"row_truth", truth_table.Close()}};
    const auto& st = engine.Stats();
    std::lock_guard<std::mutex> lock(mutex);
    if (!batch.empty()) render_queue.push_back(std::move(batch));
    metadata_tables = meta;
    evaluation_tables = evalj;
    timing_stats = {{"samples", st.samples}, {"scan_edges", st.scan_edges}, {"odometer_edges", st.odo_edges},
                    {"gate_events", st.gate_events}, {"triggers", st.triggers},
                    {"triggers_outside_gate", st.triggers_outside_gate}, {"rows", st.rows},
                    {"dropped_early_edge", st.dropped_early_edge}, {"dropped_no_period", st.dropped_no_period},
                    {"dropped_reverse", st.dropped_reverse}, {"dropped_overrun", st.dropped_overrun},
                    {"dropped_stream_end", st.dropped_stream_end}};
    timing_done = true;
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
      }
      RenderedBatch b;
      b.first_sequence = jobs.front().record.sequence;
      b.rows = jobs.size();
      b.last_center = jobs.back().record.t_center;
      const auto t0 = Clock::now();
      renderer->Render(jobs, b.pixels, b.hits);
      const double dt = Seconds(t0, Clock::now());
      if (config.debug_delay_per_batch_s > 0)
        std::this_thread::sleep_for(std::chrono::duration<double>(config.debug_delay_per_batch_s));
      std::unique_lock<std::mutex> lock(mutex);
      render_seconds += dt;
      const size_t bytes = b.Bytes();
      cv.wait(lock, [&] {
        return failed || write_queue.empty() || write_queue_bytes + bytes <= config.write_queue_bytes;
      });
      if (failed) return;
      write_queue_bytes += bytes;
      write_queue_peak = std::max(write_queue_peak, write_queue_bytes);
      write_queue.push_back(std::move(b));
      cv.notify_all();
    }
    std::lock_guard<std::mutex> lock(mutex);
    render_done = true;
    cv.notify_all();
  } catch (...) {
    Fail(std::current_exception());
  }
}

void Pipeline::Impl::WriteLoop() {
  try {
    BlockWriter blocks(root / "raw", config.width, config.block_rows);
    const size_t columns = debug_columns.size();
    nlohmann::json dtype = nlohmann::json::array({{"sequence", "<i8"}});
    if (columns) dtype.push_back({"hits", "<f8", {columns, 2}});
    TableWriter hit_table(root / "evaluation" / "debug_hits.bin", dtype, 8 + columns * 16);
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
      for (size_t i = 0; i < b.rows; ++i) {
        const int64_t seq = b.first_sequence + static_cast<int64_t>(i);
        std::memcpy(record.data(), &seq, 8);
        if (columns) std::memcpy(record.data() + 8, b.hits.data() + i * columns * 2, columns * 16);
        hit_table.Append(record.data(), 1);
      }
      const auto t1 = Clock::now();
      latest_written_center = b.last_center;
      std::lock_guard<std::mutex> lock(mutex);
      write_seconds += Seconds(t0, t1);
      ++batches;
      progress.push_back({{"wall_s", Seconds(start_wall, t1)}, {"rows_written", blocks.RowsWritten()},
                          {"sim_time_written", b.last_center}, {"sim_time_pushed", latest_pushed.load()}});
    }
    nlohmann::json raw = blocks.Close();
    nlohmann::json hits = hit_table.Close();
    hits["columns"] = debug_columns;
    std::lock_guard<std::mutex> lock(mutex);
    raw_index = raw;
    evaluation_tables["debug_hits"] = hits;
  } catch (...) {
    Fail(std::current_exception());
  }
}

nlohmann::json Pipeline::Wait() {
  auto& s = *impl_;
  if (s.waited) throw std::logic_error("Pipeline::Wait called twice");
  s.waited = true;
  s.timing_thread.join();
  s.render_thread.join();
  s.write_thread.join();
  const auto end = Clock::now();
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
  const double wall = Seconds(s.start_wall, end);
  const double imaging_wall = s.have_first ? Seconds(s.first_push_wall, end) : 0;
  const double producer_wall = s.have_first && s.finish_called ? Seconds(s.first_push_wall, s.finish_called_wall) : 0;
  const int64_t rows = s.raw_index.at("rows").get<int64_t>();
  const double sim_first = s.first_pushed;
  double sim_last = sim_first;
  if (!s.progress.empty()) sim_last = s.progress.back().at("sim_time_written").get<double>();
  const nlohmann::json performance = {
      {"wall_seconds", wall},
      {"rows", rows},
      {"average_rows_per_wall_second", wall > 0 ? rows / wall : 0},
      {"render_seconds", s.render_seconds},
      {"render_rows_per_second", s.render_seconds > 0 ? rows / s.render_seconds : 0},
      {"write_seconds", s.write_seconds},
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
      {"batches", s.batches},
      {"batch_rows", s.config.batch_rows},
      {"note", "wall_seconds starts after the backend self-check; the rates start at the first pose sample"}};
  WriteJsonAtomic(s.root / "logs" / "performance.json", {{"summary", performance}, {"progress", s.progress}});
  WriteJsonAtomic(s.root / "metadata" / "manifest.json", s.metadata_tables);
  WriteJsonAtomic(s.root / "evaluation" / "manifest.json", s.evaluation_tables);
  WriteJsonAtomic(s.root / "raw" / "index.json", s.raw_index);
  const nlohmann::json summary = {{"schema", "ssb.session.v1"},
                                  {"status", "complete"},
                                  {"pose_source", s.options.pose_source},
                                  {"rows", rows},
                                  {"timing", s.timing_stats},
                                  {"performance", performance}};
  WriteJsonAtomic(s.root / "session.json", summary);
  return summary;
}

}  // namespace ssb
