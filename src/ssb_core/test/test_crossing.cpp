#include <gtest/gtest.h>

#include <cmath>

#include "ssb_core/crossing.hpp"

using ssb::Crossing;
using ssb::FindCrossings;
using ssb::Hermite;
using ssb::LatticeIndex;

TEST(Crossing, LinearMotionHitsEveryLevelAtItsExactTime) {
  const double w = 3.4, d = 2 * M_PI / 10000, H = 1e-3;
  Hermite h{0, H, 0.1 * d, 0.1 * d + w * H, w, w};
  std::vector<Crossing> out;
  const int64_t end = FindCrossings(h, 0, d, 0, out);
  EXPECT_EQ(end, LatticeIndex(h.p1, 0, d));
  ASSERT_EQ(static_cast<int64_t>(out.size()), end);
  for (size_t i = 0; i < out.size(); ++i) {
    EXPECT_EQ(out[i].index, static_cast<int64_t>(i) + 1);
    EXPECT_EQ(out[i].dir, 1);
    EXPECT_NEAR(out[i].t, ((i + 1) * d - 0.1 * d) / w, 1e-15);
  }
}

TEST(Crossing, StartingExactlyOnALevelIsNotACrossing) {
  const double d = 0.5;
  Hermite h{0, 1, 3 * d, 3 * d + 0.2, 0.2, 0.2};
  std::vector<Crossing> out;
  EXPECT_EQ(FindCrossings(h, 0, d, LatticeIndex(h.p0, 0, d), out), 3);
  EXPECT_TRUE(out.empty());
}

TEST(Crossing, ConstantAccelerationIsExact) {
  // p = a t^2 / 2 is reproduced exactly by the Hermite segment.
  const double a = 2.0, d = 1e-3, H = 0.1;
  Hermite h{0, H, 0, 0.5 * a * H * H, 0, a * H};
  std::vector<Crossing> out;
  FindCrossings(h, 0, d, 0, out);
  ASSERT_FALSE(out.empty());
  for (const auto& c : out) EXPECT_NEAR(c.t, std::sqrt(2 * c.index * d / a), 1e-12);
}

TEST(Crossing, ReversalWithinOneSegmentReportsBothDirections) {
  // Up then back down to the start value: p(u) = 3u^2 - ... via m0 > 0, m1 < 0, p1 = p0.
  const double d = 0.01;
  Hermite h{0, 1, 0.005, 0.005, 0.3, -0.3};
  std::vector<Crossing> out;
  const int64_t end = FindCrossings(h, 0, d, 0, out);
  EXPECT_EQ(end, 0);
  ASSERT_FALSE(out.empty());
  ASSERT_EQ(out.size() % 2, 0u);
  const size_t half = out.size() / 2;
  for (size_t i = 0; i < half; ++i) EXPECT_EQ(out[i].dir, 1);
  for (size_t i = half; i < out.size(); ++i) EXPECT_EQ(out[i].dir, -1);
  for (size_t i = 1; i < out.size(); ++i) EXPECT_GT(out[i].t, out[i - 1].t);
  EXPECT_EQ(out[half - 1].index, out[half].index + 1);  // the top level is crossed twice
}

TEST(Crossing, ChainedSegmentsAgreeWithTheLatticeAtEverySample) {
  const double d = 2 * M_PI / 10000, dt = 1e-3;
  auto p = [](double t) { return 2.0 * t + 0.3 * std::sin(7 * t); };
  auto m = [](double t) { return 2.0 + 2.1 * std::cos(7 * t); };
  int64_t cur = LatticeIndex(p(0), 0.2, d);
  double last_t = -1;
  std::vector<Crossing> out;
  for (int i = 0; i < 2000; ++i) {
    const double t0 = i * dt, t1 = (i + 1) * dt;
    out.clear();
    cur = FindCrossings({t0, t1, p(t0), p(t1), m(t0), m(t1)}, 0.2, d, cur, out);
    EXPECT_EQ(cur, LatticeIndex(p(t1), 0.2, d));
    for (const auto& c : out) {
      EXPECT_GT(c.t, t0);
      EXPECT_LE(c.t, t1);
      EXPECT_GT(c.t, last_t);
      last_t = c.t;
    }
  }
}
