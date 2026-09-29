#pragma once
#include <cstdint>
#include <vector>

namespace ssb {

// Cubic Hermite segment between two pose-stream samples (value and derivative at
// both ends). Values at t0 and t1 are returned exactly as sampled so that adjacent
// segments agree bit for bit at their shared sample.
struct Hermite {
  double t0 = 0, t1 = 0, p0 = 0, p1 = 0, m0 = 0, m1 = 0;
  double Value(double t) const;
  double Slope(double t) const;
  // Offset from p0 at normalised time u in [0, 1]; avoids cancellation on large p0.
  double Delta(double u) const;
};

// A change of floor((p - offset) / spacing). `index` is the lattice index after the
// crossing; dir is +1 for an upward and -1 for a downward crossing.
struct Crossing {
  double t = 0;
  int64_t index = 0;
  int dir = 0;
};

int64_t LatticeIndex(double p, double offset, double spacing);

// Appends, in time order, every crossing inside (t0, t1] of a segment whose value at
// t0 lies in lattice cell `current`. Returns the cell at t1. Handles non-monotonic
// segments (stop, reversal) by splitting at the derivative roots.
int64_t FindCrossings(const Hermite& h, double offset, double spacing, int64_t current,
                      std::vector<Crossing>& out);

}  // namespace ssb
