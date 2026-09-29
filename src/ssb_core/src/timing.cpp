#include "ssb_core/timing.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ssb {
namespace {
constexpr double kTwoPi = 6.28318530717958647692;
// Timing resolution granted to the camera's line-period limit.
constexpr double kLinePeriodTolerance = 1e-9;

int64_t FloorDiv(int64_t a, int64_t b) { return a / b - ((a % b != 0) && ((a < 0) != (b < 0))); }
}  // namespace

void TimingOutput::Clear() {
  scan_edges.clear();
  odo_edges.clear();
  gates.clear();
  dropped.clear();
  rows.clear();
}

TimingEngine::TimingEngine(const Config& config) : config_(config) {
  config_.Validate();
  multiply_ = config_.rescale_multiply;
  divide_ = config_.rescale_divide;
  scan_spacing_ = kTwoPi / config_.CountsPerRev();
  scan_offset_ = config_.start_theta_rad + config_.truth.scan_encoder_zero_rad;
  odo_spacing_ = kTwoPi / (double(config_.odo_ppr) * config_.odo_edges_per_cycle * config_.odo_gear_ratio);
  gate_start_offset_ = config_.gate_start_rad + config_.truth.gate_start_offset_rad;
  gate_end_offset_ = config_.gate_end_rad + config_.truth.gate_end_offset_rad;
  center_delay_ = config_.trigger_delay_s + 0.5 * config_.exposure_s;
  finalize_delay_ = config_.trigger_delay_s + config_.exposure_s;
  min_line_period_ = 1.0 / config_.max_line_rate_hz - kLinePeriodTolerance;
}

double TimingEngine::SubtickTime(const Pending& p, int j) const {
  return p.t_edge + j * p.period / multiply_;
}

void TimingEngine::Push(const PoseSample& s, TimingOutput& out) {
  for (double v : {s.t, s.x, s.v, s.theta, s.omega, s.wheel, s.wheel_omega})
    if (!std::isfinite(v)) throw std::invalid_argument("non-finite pose sample");
  ++stats_.samples;
  if (!have_sample_) {
    have_sample_ = true;
    last_ = s;
    window_.push_back(s);
    scan_abs_ = scan_initial_ = LatticeIndex(s.theta, scan_offset_, scan_spacing_);
    odo_abs_ = odo_initial_ = LatticeIndex(s.wheel, 0.0, odo_spacing_);
    gate_start_idx_ = LatticeIndex(s.theta, gate_start_offset_, kTwoPi);
    gate_end_idx_ = LatticeIndex(s.theta, gate_end_offset_, kTwoPi);
    const bool open = gate_start_idx_ == gate_end_idx_ + 1;
    gate_history_.push_back({s.t, open, gate_start_idx_});
    // Nothing is interpolated before the first edge: lattice 0..63 awaits it.
    no_period_first_subtick_ = 0;
    no_period_since_ = s.t;
    return;
  }
  if (!(s.t > last_.t)) throw std::invalid_argument("pose samples must have increasing time");

  const Hermite theta{last_.t, s.t, last_.theta, s.theta, last_.omega, s.omega};
  const Hermite wheel{last_.t, s.t, last_.wheel, s.wheel, last_.wheel_omega, s.wheel_omega};
  std::vector<Crossing> scan, odo, starts, ends;
  scan_abs_ = FindCrossings(theta, scan_offset_, scan_spacing_, scan_abs_, scan);
  odo_abs_ = FindCrossings(wheel, 0.0, odo_spacing_, odo_abs_, odo);
  gate_start_idx_ = FindCrossings(theta, gate_start_offset_, kTwoPi, gate_start_idx_, starts);
  gate_end_idx_ = FindCrossings(theta, gate_end_offset_, kTwoPi, gate_end_idx_, ends);

  for (const auto& c : odo) {
    out.odo_edges.push_back({c.t, c.index - odo_initial_, c.dir});
    ++stats_.odo_edges;
  }

  // Gate events: a start crossed upward opens revolution k; an end crossed upward
  // closes it. Downward crossings undo the transition of the level they cross.
  std::vector<GateRecord> gates;
  for (const auto& c : starts)
    gates.push_back({c.t, c.dir > 0 ? c.index : c.index + 1, kGateStart, c.dir});
  for (const auto& c : ends)
    gates.push_back({c.t, c.dir > 0 ? c.index : c.index + 1, kGateEnd, c.dir});
  std::stable_sort(gates.begin(), gates.end(), [](const GateRecord& a, const GateRecord& b) { return a.t < b.t; });
  for (const auto& g : gates) {
    const bool open = (g.kind == kGateStart) == (g.dir > 0);
    gate_history_.push_back({g.t, open, g.revolution});
    out.gates.push_back(g);
    ++stats_.gate_events;
  }

  for (const auto& e : scan) {
    EmitPendingBefore(e.t, false);
    ProcessScanEdge(e, out);
  }
  EmitPendingBefore(s.t, true);

  last_ = s;
  window_.push_back(s);
  FinalizeTriggers(s.t, out);
  FinalizeDropped(s.t, out);
  PruneHistory();
}

void TimingEngine::EmitPendingBefore(double limit, bool inclusive) {
  while (pending_.active) {
    if (pending_.next_j >= multiply_) {
      pending_.active = false;
      break;
    }
    const double t = SubtickTime(pending_, pending_.next_j);
    if (inclusive ? t > limit : t >= limit) break;
    const int64_t subtick = pending_.count * multiply_ + pending_.next_j;
    triggers_.push_back({subtick / divide_, t});
    pending_.next_j += divide_;
  }
}

void TimingEngine::DropLattice(int64_t first, int64_t last, double t_lo, double t_hi, int32_t reason,
                               TimingOutput&) {
  for (int64_t s = FloorDiv(first + divide_ - 1, divide_) * divide_; s <= last; s += divide_)
    dropped_waiting_.push_back({s / divide_, t_lo, t_hi, reason, 0});
}

void TimingEngine::DropPending(double t_hi, int32_t reason, TimingOutput& out) {
  if (!pending_.active) return;
  if (pending_.next_j < multiply_) {
    const int64_t base = pending_.count * multiply_;
    DropLattice(base + pending_.next_j, base + multiply_ - 1, pending_.t_edge, t_hi, reason, out);
  }
  pending_.active = false;
}

void TimingEngine::ProcessScanEdge(const Crossing& edge, TimingOutput& out) {
  const int64_t count = edge.index - scan_initial_;
  out.scan_edges.push_back({edge.t, count, edge.dir});
  ++stats_.scan_edges;
  if (edge.dir > 0 && count > high_water_) {
    if (count != high_water_ + 1) throw std::logic_error("encoder skipped a count");
    DropPending(edge.t, kDropEarlyEdge, out);
    const int64_t subtick = count * multiply_;
    if (no_period_first_subtick_ >= 0) {
      DropLattice(no_period_first_subtick_, subtick - 1, no_period_since_, edge.t, kDropNoPeriod, out);
      no_period_first_subtick_ = -1;
    }
    if (subtick % divide_ == 0) triggers_.push_back({subtick / divide_, edge.t});
    high_water_ = count;
    const double period = edge.t - prev_edge_t_;
    const bool valid = have_prev_edge_ && prev_edge_dir_ > 0 && prev_edge_count_ == count - 1 &&
                       period > 0 && period <= config_.rescale_max_period_s;
    if (valid) {
      int j = 1;
      while ((subtick + j) % divide_ != 0) ++j;
      pending_ = {true, count, edge.t, period, j};
    } else {
      pending_.active = false;
      no_period_first_subtick_ = subtick + 1;
      no_period_since_ = edge.t;
    }
  } else {
    DropPending(edge.t, kDropReverse, out);
  }
  have_prev_edge_ = true;
  prev_edge_t_ = edge.t;
  prev_edge_count_ = count;
  prev_edge_dir_ = edge.dir;
}

bool TimingEngine::GateAt(double t, int64_t* revolution) const {
  auto it = std::upper_bound(gate_history_.begin(), gate_history_.end(), t,
                             [](double value, const GateState& g) { return value < g.t; });
  const GateState& g = it == gate_history_.begin() ? gate_history_.front() : *(it - 1);
  if (revolution) *revolution = g.revolution;
  return g.open;
}

PoseSample TimingEngine::PoseAt(double t) const {
  if (window_.empty() || t < window_.front().t || t > window_.back().t)
    throw std::logic_error("pose requested outside the retained stream window");
  size_t i = 0;
  while (i + 1 < window_.size() && window_[i + 1].t < t) ++i;
  if (i + 1 == window_.size()) return window_[i];
  const PoseSample &a = window_[i], &b = window_[i + 1];
  const Hermite x{a.t, b.t, a.x, b.x, a.v, b.v};
  const Hermite th{a.t, b.t, a.theta, b.theta, a.omega, b.omega};
  const Hermite w{a.t, b.t, a.wheel, b.wheel, a.wheel_omega, b.wheel_omega};
  return {t, x.Value(t), x.Slope(t), th.Value(t), th.Slope(t), w.Value(t), w.Slope(t)};
}

void TimingEngine::FinalizeTriggers(double covered, TimingOutput& out) {
  while (!triggers_.empty() && triggers_.front().t + finalize_delay_ <= covered) {
    const Trigger tr = triggers_.front();
    triggers_.pop_front();
    ++stats_.triggers;
    const double center = tr.t + center_delay_;
    int64_t revolution = 0;
    if (!GateAt(center, &revolution)) {
      ++stats_.triggers_outside_gate;
      continue;
    }
    if (have_accepted_ && tr.t - last_accepted_t_ < min_line_period_) {
      out.dropped.push_back({tr.row, center, center, kDropOverrun, 1});
      ++stats_.dropped_overrun;
      continue;
    }
    have_accepted_ = true;
    last_accepted_t_ = tr.t;
    out.rows.push_back({{sequence_++, tr.row, revolution, tr.t, center}, PoseAt(center)});
    ++stats_.rows;
  }
}

void TimingEngine::FinalizeDropped(double covered, TimingOutput& out) {
  while (!dropped_waiting_.empty() && dropped_waiting_.front().t_hi <= covered) {
    DroppedRowRecord d = dropped_waiting_.front();
    dropped_waiting_.pop_front();
    d.gated = GateAt(d.t_lo, nullptr) || GateAt(d.t_hi, nullptr);
    switch (d.reason) {
      case kDropEarlyEdge: ++stats_.dropped_early_edge; break;
      case kDropNoPeriod: ++stats_.dropped_no_period; break;
      case kDropReverse: ++stats_.dropped_reverse; break;
      default: ++stats_.dropped_stream_end; break;
    }
    out.dropped.push_back(d);
  }
}

void TimingEngine::PruneHistory() {
  const double needed = triggers_.empty() ? last_.t : triggers_.front().t + center_delay_;
  while (window_.size() >= 2 && window_[1].t <= needed) window_.pop_front();
}

void TimingEngine::Finish(TimingOutput& out) {
  if (!have_sample_) return;
  const double end = last_.t;
  while (!triggers_.empty()) {
    const Trigger tr = triggers_.front();
    triggers_.pop_front();
    dropped_waiting_.push_back({tr.row, tr.t, end, kDropStreamEnd, 0});
  }
  DropPending(end, kDropStreamEnd, out);
  if (no_period_first_subtick_ >= 0) {
    DropLattice(no_period_first_subtick_, (high_water_ + 1) * multiply_ - 1, no_period_since_, end,
                kDropStreamEnd, out);
    no_period_first_subtick_ = -1;
  }
  // Stream-end drops may be out of time order with earlier waiting entries.
  for (auto& d : dropped_waiting_) d.t_hi = std::min(d.t_hi, end), d.t_lo = std::min(d.t_lo, end);
  FinalizeDropped(end, out);
}

}  // namespace ssb
