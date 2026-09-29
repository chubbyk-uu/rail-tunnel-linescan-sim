#include "ssb_core/crossing.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ssb {
namespace {

struct Poly {
  double b, c, d;  // Delta(u) = b u + c u^2 + d u^3
};

Poly Coefficients(const Hermite& h) {
  const double H = h.t1 - h.t0, dp = h.p1 - h.p0;
  return {H * h.m0, 3 * dp - 2 * H * h.m0 - H * h.m1, -2 * dp + H * h.m0 + H * h.m1};
}

double ValueAt(const Hermite& h, double u) {
  if (u <= 0) return h.p0;
  if (u >= 1) return h.p1;
  return h.p0 + h.Delta(u);
}

double TimeAt(const Hermite& h, double u) {
  if (u >= 1) return h.t1;
  return h.t0 + u * (h.t1 - h.t0);
}

// Roots of 3 d u^2 + 2 c u + b in the open interval (0, 1), ascending.
std::vector<double> CriticalPoints(const Poly& p) {
  std::vector<double> roots;
  const double A = 3 * p.d, B = 2 * p.c, C = p.b;
  const double scale = std::max({std::abs(A), std::abs(B), std::abs(C)});
  if (scale == 0) return roots;
  if (std::abs(A) <= 1e-14 * scale) {
    if (B != 0) roots.push_back(-C / B);
  } else {
    const double disc = B * B - 4 * A * C;
    if (disc >= 0) {
      const double q = -0.5 * (B + std::copysign(std::sqrt(disc), B));
      if (q != 0) roots.push_back(q / A);
      if (q != 0) roots.push_back(C / q);
      else roots.push_back(0);
    }
  }
  std::vector<double> inside;
  for (double r : roots)
    if (std::isfinite(r) && r > 0 && r < 1) inside.push_back(r);
  std::sort(inside.begin(), inside.end());
  inside.erase(std::unique(inside.begin(), inside.end()), inside.end());
  return inside;
}

}  // namespace

double Hermite::Delta(double u) const {
  const Poly p = Coefficients(*this);
  if (u >= 1) return p1 - p0;
  return ((p.d * u + p.c) * u + p.b) * u;
}

double Hermite::Value(double t) const {
  if (t == t0) return p0;
  if (t == t1) return p1;
  return p0 + Delta((t - t0) / (t1 - t0));
}

double Hermite::Slope(double t) const {
  const Poly p = Coefficients(*this);
  const double H = t1 - t0, u = (t - t0) / H;
  return (p.b + (2 * p.c + 3 * p.d * u) * u) / H;
}

int64_t LatticeIndex(double p, double offset, double spacing) {
  const double k = std::floor((p - offset) / spacing);
  if (!std::isfinite(k) || std::abs(k) > 9.0e15) throw std::range_error("lattice index out of range");
  return static_cast<int64_t>(k);
}

int64_t FindCrossings(const Hermite& h, double offset, double spacing, int64_t current,
                      std::vector<Crossing>& out) {
  if (!(h.t1 > h.t0) || !(spacing > 0) || !std::isfinite(h.p0) || !std::isfinite(h.p1) ||
      !std::isfinite(h.m0) || !std::isfinite(h.m1))
    throw std::invalid_argument("invalid Hermite segment");
  std::vector<double> breaks = {0.0};
  for (double r : CriticalPoints(Coefficients(h))) breaks.push_back(r);
  breaks.push_back(1.0);

  auto index = [&](double u) { return LatticeIndex(ValueAt(h, u), offset, spacing); };
  for (size_t piece = 0; piece + 1 < breaks.size(); ++piece) {
    double lo = breaks[piece];
    const double hi_end = breaks[piece + 1];
    const int64_t end = index(hi_end);
    const int dir = end > current ? 1 : -1;
    // Upward: first u where index >= target. Downward: first u where index < target + 1.
    while (current != end) {
      const int64_t target = current + dir;
      auto reached = [&](double u) { return dir > 0 ? index(u) >= target : index(u) <= target; };
      double a = lo, b = hi_end;
      for (int i = 0; i < 200; ++i) {
        const double mid = 0.5 * (a + b);
        if (mid <= a || mid >= b) break;
        (reached(mid) ? b : a) = mid;
      }
      out.push_back({TimeAt(h, b), target, dir});
      current = target;
      lo = b;
    }
  }
  return current;
}

}  // namespace ssb
