#pragma once
#include <cstdint>
#include <deque>
#include <vector>

#include "ssb_core/config.hpp"
#include "ssb_core/crossing.hpp"
#include "ssb_core/records.hpp"

namespace ssb {

// A recorded exposure plus the true pose at its centre, handed to imaging.
struct RowJob {
  RowRecord record;
  PoseSample pose;  // true pose interpolated at record.t_center
};

struct TimingOutput {
  std::vector<EdgeRecord> scan_edges, odo_edges;
  std::vector<GateRecord> gates;
  std::vector<DroppedRowRecord> dropped;
  std::vector<RowJob> rows;
  void Clear();
};

struct TimingStats {
  int64_t samples = 0, scan_edges = 0, odo_edges = 0, gate_events = 0;
  int64_t triggers = 0, triggers_outside_gate = 0, rows = 0;
  int64_t dropped_early_edge = 0, dropped_no_period = 0, dropped_reverse = 0;
  int64_t dropped_overrun = 0, dropped_stream_end = 0;
};

// Streaming hardware timing model (DESIGN.md §4.2, §8.1): quadrature edges from the
// true scan axis, x64 sub-ticks by causal period estimate, /7 row triggers, gate
// sensors on the true axis angle, and camera line-rate limit. Output depends only on
// the sequence of samples, never on how Push calls are grouped in time.
//
// Rescaler model (stage A, not a firmware replica): every forward edge beyond the
// high-water count c is sub-tick 64c. With T the time since edge c-1, sub-ticks
// 64c+j (j=1..63) fall at t_c + jT/64 if they precede the next edge; the rest are
// dropped. No sub-ticks are interpolated without a valid T (<= max_period_s), after a
// reversal, or below the high-water count.
class TimingEngine {
 public:
  explicit TimingEngine(const Config& config);
  void Push(const PoseSample& sample, TimingOutput& out);
  // End of stream: pending lattice rows are reported as dropped (stream end).
  void Finish(TimingOutput& out);
  const TimingStats& Stats() const { return stats_; }
  double CoveredTime() const { return have_sample_ ? last_.t : 0; }

 private:
  struct Pending {  // scheduled interpolated sub-ticks after a forward edge
    bool active = false;
    int64_t count = 0;  // edge count c
    double t_edge = 0, period = 0;
    int next_j = 1;
  };
  struct Trigger {
    int64_t row;
    double t;
  };
  struct GateState {
    double t;
    bool open;
    int64_t revolution;
  };

  void ProcessScanEdge(const Crossing& edge, TimingOutput& out);
  void EmitPendingBefore(double limit, bool inclusive);
  void DropPending(double t_hi, int32_t reason, TimingOutput& out);
  void DropLattice(int64_t first_subtick, int64_t last_subtick, double t_lo, double t_hi, int32_t reason,
                   TimingOutput& out);
  void FinalizeTriggers(double covered, TimingOutput& out);
  void FinalizeDropped(double covered, TimingOutput& out);
  bool GateAt(double t, int64_t* revolution) const;
  PoseSample PoseAt(double t) const;
  double SubtickTime(const Pending& p, int j) const;
  void PruneHistory();

  Config config_;
  int multiply_, divide_;
  double scan_spacing_, scan_offset_, odo_spacing_;
  double gate_start_offset_, gate_end_offset_;
  double center_delay_, finalize_delay_, min_line_period_;

  bool have_sample_ = false;
  PoseSample last_;
  std::deque<PoseSample> window_;  // samples still needed for pose interpolation
  int64_t scan_abs_ = 0, scan_initial_ = 0, odo_abs_ = 0, odo_initial_ = 0;
  int64_t gate_start_idx_ = 0, gate_end_idx_ = 0;

  // Rescaler state.
  int64_t high_water_ = 0;
  bool have_prev_edge_ = false;
  double prev_edge_t_ = 0;
  int64_t prev_edge_count_ = 0;
  int prev_edge_dir_ = 0;
  Pending pending_;
  double no_period_since_ = 0;
  int64_t no_period_first_subtick_ = -1;  // lattice awaiting the next edge, or -1

  std::deque<Trigger> triggers_;
  std::deque<DroppedRowRecord> dropped_waiting_;  // gated flag resolved when covered
  std::vector<GateState> gate_history_;
  bool have_accepted_ = false;
  double last_accepted_t_ = 0;
  int64_t sequence_ = 0;
  TimingStats stats_;
};

}  // namespace ssb
