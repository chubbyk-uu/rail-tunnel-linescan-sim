#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

#include "ssb_core/kinematic.hpp"
#include "ssb_core/timing.hpp"

using namespace ssb;

TEST(StageB, LensProjectionAndSynchronizedMotionUseUpdatedBaseline) {
  const auto c = Config::Load(std::string(SSB_CONFIG_DIR) + "/stage_b.yaml");
  const double f = 0.09, R = 2.75;
  EXPECT_NEAR(c.FocalLength(), f * R / (R - f), 1e-12);
  EXPECT_NEAR(c.RowsPerRev(), 10000.0 * 128 / 15, 1e-9);
  EXPECT_NEAR(c.advance_per_rev_m * c.NominalOmega() / (2 * M_PI), 0.2, 1e-12);
  EXPECT_NEAR(c.NominalOmega() * 60 / (2 * M_PI), 20, 1e-12);
}

TEST(Config, DistanceStopIsPublicValidatedAndDoesNotAffectOpticalIdentity) {
  auto c=Config::Load(std::string(SSB_CONFIG_DIR)+"/stage_b.yaml");
  c.contact_enabled=true;
  c.odo_left_calibrated=c.odo_right_calibrated=c.odo_left_true=c.odo_right_true=.08;
  const auto signature=c.OpticalSignature();
  c.distance_stop.target_m=3.;c.distance_stop.timeout_s=40.;
  EXPECT_NO_THROW(c.Validate());
  EXPECT_DOUBLE_EQ(c.ObservableJson()["motion"]["distance_stop"]["target_m"],3.);
  EXPECT_EQ(signature,c.OpticalSignature());
  c.distance_stop.timeout_s=5.;EXPECT_THROW(c.Validate(),std::runtime_error);
  c.distance_stop.timeout_s=40.;c.distance_stop.hold_s=0.;EXPECT_THROW(c.Validate(),std::runtime_error);
  c.distance_stop.hold_s=.5;c.contact_enabled=false;EXPECT_THROW(c.Validate(),std::runtime_error);
}

TEST(StageC, WallTargetIsPublicAndDoesNotChangeOpticalIdentity) {
  auto c=Config::Load(std::string(SSB_CONFIG_DIR)+"/stage_b.yaml");
  const auto signature=c.OpticalSignature();
  c.inspection={{"schema","ssb.wall_target.v1"},{"target_x_m",{0.,20.}},
                {"theta_rad",{-2*M_PI/3,2*M_PI/3}},{"grid_pitch_m",.0002},{"guard_m",.01}};
  EXPECT_NO_THROW(c.Validate());
  EXPECT_EQ(c.ObservableJson().at("inspection"),c.inspection);
  EXPECT_EQ(c.OpticalSignature(),signature);
  const auto parsed=Config::Parse(c.source_text+
    "\ninspection:\n  schema: ssb.wall_target.v1\n  target_x_m: [0, 20]\n"
    "  theta_rad: [-2.0943951023931953, 2.0943951023931953]\n  grid_pitch_m: 0.0002\n  guard_m: 0.01\n");
  EXPECT_EQ(parsed.ObservableJson().at("inspection"),c.inspection);
  c.inspection["target_x_m"]={20.,0.};
  EXPECT_THROW(c.Validate(),std::runtime_error);
  c.inspection["target_x_m"]={0.,20.};c.inspection["grid_pitch_m"]=0.;
  EXPECT_THROW(c.Validate(),std::runtime_error);
}

namespace {

Config BaseConfig() { return Config::Load(std::string(SSB_CONFIG_DIR) + "/stage_a.yaml"); }

struct TimingRun {
  TimingOutput all;
  TimingStats stats;
};

TimingRun RunSamples(const Config& c, const std::vector<PoseSample>& samples) {
  TimingEngine engine(c);
  TimingRun run;
  TimingOutput out;
  auto keep = [&] {
    auto append = [](auto& to, const auto& from) { to.insert(to.end(), from.begin(), from.end()); };
    append(run.all.scan_edges, out.scan_edges);
    append(run.all.odo_edges, out.odo_edges);
    append(run.all.gates, out.gates);
    append(run.all.dropped, out.dropped);
    append(run.all.rows, out.rows);
    out.Clear();
  };
  for (const auto& s : samples) {
    engine.Push(s, out);
    keep();
  }
  engine.Finish(out);
  keep();
  run.stats = engine.Stats();
  return run;
}

// Every lattice row appears at most once, and inside each segment the recorded rows
// plus gated drops form one contiguous run: nothing inside the gate goes missing.
void CheckAccounting(const TimingRun& run) {
  std::set<int64_t> seen;
  for (const auto& r : run.all.rows) EXPECT_TRUE(seen.insert(r.record.row).second) << r.record.row;
  std::set<int64_t> gated_drops;
  for (const auto& d : run.all.dropped) {
    EXPECT_TRUE(seen.insert(d.row).second) << "row both recorded/dropped or dropped twice: " << d.row;
    if (d.gated) gated_drops.insert(d.row);
  }
  std::map<int64_t, std::vector<int64_t>> segments;
  for (size_t i = 0; i < run.all.rows.size(); ++i) {
    const auto& r = run.all.rows[i].record;
    EXPECT_EQ(r.sequence, static_cast<int64_t>(i));
    if (i) {
      EXPECT_GT(r.t_trigger, run.all.rows[i - 1].record.t_trigger);
      EXPECT_GT(r.row, run.all.rows[i - 1].record.row);
    }
    segments[r.segment].push_back(r.row);
  }
  for (const auto& [k, rows] : segments) {
    std::set<int64_t> have(rows.begin(), rows.end());
    for (int64_t r = rows.front(); r <= rows.back(); ++r)
      EXPECT_TRUE(have.count(r) || gated_drops.count(r)) << "segment " << k << " lost row " << r;
  }
}

}  // namespace

TEST(StageB, StartsDownAndExposesOnlyAfterRightLowerGate) {
  auto c = Config::Load(std::string(SSB_CONFIG_DIR) + "/stage_b.yaml");
  EXPECT_NEAR(c.start_theta_rad, M_PI, 1e-12);
  c.profile = {{0., 1.}, {3., 1.}};
  const auto run = RunSamples(c, KinematicSource(c).Sample());
  ASSERT_EQ(run.all.gates.size(), 2u);
  EXPECT_EQ(run.all.gates[0].kind, kGateStart);
  EXPECT_EQ(run.all.gates[1].kind, kGateEnd);
  EXPECT_NEAR(run.all.gates[0].t, .5, 1e-10); // 60 degrees at 20 rpm
  EXPECT_NEAR(run.all.gates[1].t, 2.5, 1e-10);
  ASSERT_FALSE(run.all.rows.empty());
  EXPECT_LT(std::sin(run.all.rows.front().pose.theta), 0.); // -y: vehicle right
  EXPECT_GT(std::sin(run.all.rows.back().pose.theta), 0.);  // +y: vehicle left
  for (const auto& row : run.all.rows) {
    EXPECT_GE(row.record.t_center, .5);
    EXPECT_LT(row.record.t_center, 2.5);
  }
  CheckAccounting(run);
}

TEST(Timing, ConstantSpeedGivesUniformRowsLockedToTheEncoderLattice) {
  Config c = BaseConfig();
  c.profile = {{0, 1}, {4.0, 1}};
  const TimingRun run = RunSamples(c, KinematicSource(c).Sample());
  CheckAccounting(run);
  const double omega = c.NominalOmega(), delta = 2 * M_PI / c.CountsPerRev();
  const double row_angle = delta * c.rescale_divide / c.rescale_multiply;
  const double center_delay = c.trigger_delay_s + 0.5 * c.exposure_s;
  ASSERT_GT(run.all.rows.size(), 100000u);
  // The run ends while scanning, so only stream-end drops may be gated.
  for (const auto& d : run.all.dropped)
    if (d.reason != kDropStreamEnd) EXPECT_FALSE(d.gated) << d.row << " reason " << d.reason;

  std::map<int64_t, int64_t> per_segment;
  for (size_t i = 0; i < run.all.rows.size(); ++i) {
    const auto& job = run.all.rows[i];
    ++per_segment[job.record.segment];
    // Exposure centre angle is the lattice angle plus the centre delay.
    const double lattice = c.start_theta_rad + job.record.row * row_angle;
    EXPECT_NEAR(job.pose.theta, lattice + omega * center_delay, 1e-9);
    if (i && job.record.segment == run.all.rows[i - 1].record.segment)
      EXPECT_NEAR(job.record.t_trigger - run.all.rows[i - 1].record.t_trigger, 1 / c.line_rate_hz, 1e-11);
  }
  // Segment 1 starts at 240 deg and is complete; its row count is N*2/3 rounded either way.
  ASSERT_TRUE(per_segment.count(1));
  const double expected = c.RowsPerRev() * 2 / 3;
  EXPECT_GE(per_segment[1], std::floor(expected));
  EXPECT_LE(per_segment[1], std::ceil(expected));

  // Gate boundary: first row of segment 1 is at or after 240 deg, the lattice row
  // before it is not (exposure centre, start inclusive).
  const auto first = std::find_if(run.all.rows.begin(), run.all.rows.end(),
                                  [](const RowJob& j) { return j.record.segment == 1; });
  const double gate = c.gate_start_rad + 2 * M_PI;
  EXPECT_GE(first->pose.theta, gate);
  EXPECT_LT(c.start_theta_rad + (first->record.row - 1) * row_angle + omega * center_delay, gate);
}

TEST(Timing, AdvancePerRevolutionFollowsTheSynchronisedProfile) {
  Config c = BaseConfig();  // ramp up, cruise, ramp down
  const TimingRun run = RunSamples(c, KinematicSource(c).Sample());
  CheckAccounting(run);
  ASSERT_GT(run.all.rows.size(), 2u);
  const auto& a = run.all.rows.front().pose;
  const auto& b = run.all.rows.back().pose;
  EXPECT_NEAR((b.x - a.x) / (b.theta - a.theta) * 2 * M_PI, c.advance_per_rev_m, 1e-9);
}

TEST(Timing, StopInsideTheGateIsReportedNotSilent) {
  Config c = BaseConfig();
  // From 180 deg: cruise to ~357 deg, decelerate to rest at ~436 deg (inside the
  // 240..480 deg gate), hold, then restart.
  c.profile = {{0, 1}, {0.9, 1}, {1.7, 0}, {2.2, 0}, {3.0, 1}, {4.0, 1}};
  const TimingRun run = RunSamples(c, KinematicSource(c).Sample());
  CheckAccounting(run);
  int64_t gated_no_period = 0, gated_early = 0;
  for (const auto& d : run.all.dropped) {
    gated_no_period += d.gated && d.reason == kDropNoPeriod;
    gated_early += d.gated && d.reason == kDropEarlyEdge;
  }
  EXPECT_GT(gated_no_period + gated_early, 0);
  EXPECT_EQ(run.stats.dropped_overrun, 0);
}

TEST(Timing, ReversalNeverRepeatsARow) {
  Config c = BaseConfig();
  const double omega = c.NominalOmega();
  std::vector<PoseSample> samples;
  // Forward into the gate, back a few degrees, then forward again.
  auto theta = [&](double t) {
    const double P = 0.2, A = 0.2;  // bump slope 2*pi*A/P exceeds omega, so the axis reverses
    return c.start_theta_rad + omega * t - (t > 0.8 && t < 0.8 + P ? A * (1 - std::cos(2 * M_PI * (t - 0.8) / P)) : 0.0);
  };
  const double dt = 1e-3;
  for (int i = 0; i <= 2500; ++i) {
    const double t = i * dt, h = 1e-6;
    PoseSample s;
    s.t = t;
    s.theta = theta(t);
    s.omega = (theta(t + h) - theta(t - h)) / (2 * h);
    samples.push_back(s);
  }
  const TimingRun run = RunSamples(c, samples);
  CheckAccounting(run);
  // Sub-ticks extrapolated before the turn are already exposed, so a slow reversal
  // need not drop anything; rows must still never repeat and must resume after it.
  double last_backward = -1;
  for (const auto& e : run.all.scan_edges)
    if (e.dir < 0) last_backward = e.t;
  ASSERT_GT(last_backward, 0);
  ASSERT_FALSE(run.all.rows.empty());
  EXPECT_GT(run.all.rows.back().record.t_trigger, last_backward + 0.1);
}

TEST(Timing, TriggersFasterThanTheCameraAreDroppedAsOverrun) {
  Config c = BaseConfig();
  c.line_rate_hz = 60000;
  c.profile = {{0, 1}, {3.0, 1}};
  const TimingRun run = RunSamples(c, KinematicSource(c).Sample());
  CheckAccounting(run);
  EXPECT_GT(run.stats.dropped_overrun, 0);
  for (size_t i = 1; i < run.all.rows.size(); ++i)
    EXPECT_GE(run.all.rows[i].record.t_trigger - run.all.rows[i - 1].record.t_trigger, 1 / c.max_line_rate_hz - 2e-9);
}

TEST(Timing, OdometerCountsMatchTheWheelAngle) {
  Config c = BaseConfig();
  const auto samples = KinematicSource(c).Sample();
  const TimingRun run = RunSamples(c, samples);
  const double counts_per_rad = c.odo_ppr * c.odo_edges_per_cycle * c.odo_gear_ratio / (2 * M_PI);
  ASSERT_FALSE(run.all.odo_edges.empty());
  EXPECT_EQ(run.all.odo_edges.back().count, static_cast<int64_t>(std::floor(samples.back().wheel * counts_per_rad)));
}

TEST(Timing, BodyAndIndependentRearEncodersInterpolateThroughYawWrap) {
  auto c=BaseConfig();c.contact_enabled=true;
  TimingEngine engine(c);TimingOutput out;size_t rows=0,right=0,left=0;
  for(int i=0;i<=1000;++i){
    double t=i*.001;PoseSample p{t,3+.2*t,.2,1.5*t,1.5,2*t,2};
    p.body_valid=1;p.y=.01+.02*t;p.vy=.02;p.z=.3;
    p.yaw=std::remainder(3.13+.03*t,2*M_PI);p.yaw_rate=.03;
    p.right_wheel=t;p.right_wheel_omega=1;
    engine.Push(p,out);
    for(const auto& job:out.rows){
      ++rows;EXPECT_NEAR(job.pose.y,.01+.02*job.pose.t,1e-10);
      EXPECT_NEAR(std::remainder(job.pose.yaw-(3.13+.03*job.pose.t),2*M_PI),0,1e-10);
      EXPECT_NEAR(job.pose.right_wheel,job.pose.t,1e-10);
    }
    right+=out.right_odo_edges.size();left+=out.odo_edges.size();out.Clear();
  }
  EXPECT_GT(rows,100);EXPECT_GT(right,100);EXPECT_NEAR(double(left),2.*right,2);
}

TEST(Config, AssemblyHeightsAreRequiredInsteadOfGuessed) {
  const auto c=Config::Load(std::string(SSB_CONFIG_DIR)+"/stage_a.yaml");
  for(const std::string name:{"robot:","base_reference_z_m:","scan_axis_height_m:"}) {
    auto text=c.source_text;const auto at=text.find(name);ASSERT_NE(at,std::string::npos);
    text.replace(at,name.size(),"missing_"+name);
    EXPECT_THROW(Config::Parse(text),std::runtime_error)<<name;
  }
}

TEST(Config, PublicMissionLimitsAreValidatedWithoutChangingOpticalIdentity) {
  auto c=Config::Load(std::string(SSB_CONFIG_DIR)+"/stage_b.yaml");
  const auto signature=c.OpticalSignature();
  EXPECT_EQ(c.ObservableJson().at("mission").at("inspection_x_m"),nlohmann::json::array({0.,20.}));
  c.tunnel_x_max_m=151.5;c.mission["inspection_x_m"]={0.,150.};
  EXPECT_NO_THROW(c.Validate());EXPECT_EQ(c.OpticalSignature(),signature);
  c.mission["minimum_distance_m"]=.5;EXPECT_THROW(c.Validate(),std::runtime_error);
  c.mission["minimum_distance_m"]=1.;c.mission["inspection_x_m"]={0.,200.};
  EXPECT_THROW(c.Validate(),std::runtime_error);
}

TEST(SensorNoise, CounterStreamsHaveExpectedNormalAndLowCountPoissonMoments) {
  const int n=131072;
  for (double mean : {0.2, 2., 20., 63., 64., 2000.}) {
    double sum=0, square=0;
    for (int i=0;i<n;++i) {
      double sample=PhotonCount(float(mean),NoiseHash(uint64_t(i)+0x321123ull));
      sum+=sample;square+=sample*sample;
    }
    double average=sum/n, variance=square/n-average*average;
    EXPECT_NEAR(average,mean,.02*std::sqrt(mean)+.003);
    EXPECT_NEAR(variance,mean,.03*mean+.01);
  }
  double sum=0,square=0;
  for(int i=0;i<n;++i) {double v=NoiseNormal(NoiseHash(i),5);sum+=v;square+=v*v;}
  EXPECT_NEAR(sum/n,0,.015);EXPECT_NEAR(square/n,1,.025);
}

TEST(SensorNoise, AnalogReadoutMeanVarianceAndPositiveFixedResponse) {
  SensorNoise noise=DefaultSensorNoise();noise.enabled=true;noise.prnu_fraction=.005f;
  double gain_sum=0,gain_square=0;
  for(unsigned column=0;column<65536;++column) {
    double gain=ColumnResponse(noise,column);ASSERT_GT(gain,0.);
    gain_sum+=gain;gain_square+=gain*gain;
  }
  double gain_mean=gain_sum/65536,gain_var=gain_square/65536-gain_mean*gain_mean;
  EXPECT_NEAR(gain_mean,1.,.0001);EXPECT_NEAR(std::sqrt(gain_var),.005,.0001);
  for (float signal : {0.f, 80.f, 180.f}) {
    double sum=0,square=0;
    constexpr int n=131072;
    for(int row=0;row<n;++row) {
      double dn=SensorDN(signal,1.f,row,11,noise,8e-6);
      sum+=dn;square+=dn*dn;
    }
    double mean=sum/n,var=square/n-mean*mean;
    double expected_mean=4.+signal+8e-6*100/20;
    double expected_var=(signal*20+8e-6*100+64)/(20*20);
    EXPECT_NEAR(mean,expected_mean,.025);EXPECT_NEAR(var,expected_var,.03*expected_var+.005);
  }
  EXPECT_NE(SensorDN(80,1,1,2,noise,8e-6),SensorDN(80,1,2,1,noise,8e-6));
  noise.enabled=false;EXPECT_EQ(SensorDN(31.25,2.f,999,11,noise,1.),31.25f);
}

TEST(SensorNoise, TruthOnlyParametersAndOpticalIdentitySeparateFixedAndTemporalSeeds) {
  auto c=BaseConfig();const auto original=c.OpticalSignature();
  c.truth.sensor_noise.realization_seed=55;
  EXPECT_EQ(c.OpticalSignature(),original);
  c.truth.sensor_noise.enabled=true;
  const auto signature=c.OpticalSignature();EXPECT_NE(signature,original);
  c.truth.sensor_noise.realization_seed++;
  EXPECT_EQ(c.OpticalSignature(),signature);
  c.truth.sensor_noise.pattern_seed++;
  EXPECT_NE(c.OpticalSignature(),signature);
  const auto public_json=c.ObservableJson();
  EXPECT_TRUE(public_json.at("camera").at("sensor_noise_enabled"));
  for(const auto* name : {"pattern_seed","realization_seed","electrons_per_dn","read_noise_e","dark_current_e_per_s","bias_dn","prnu_fraction"})
    EXPECT_EQ(public_json.dump().find(name),std::string::npos);
  EXPECT_EQ(c.TruthJson().at("sensor_noise").at("realization_seed"),56);
  c.truth.sensor_noise.read_noise_e=-1;EXPECT_THROW(c.Validate(),std::runtime_error);
  c.truth.sensor_noise.read_noise_e=8;c.truth.sensor_noise.prnu_fraction=.2;EXPECT_THROW(c.Validate(),std::runtime_error);
}

TEST(SensorNoise, ParsesExplicitPrivateModelAndRejectsUnknownOrNonfiniteParameters) {
  auto c=BaseConfig();std::string text=c.source_text;
  const auto at=text.find("truth:\n");ASSERT_NE(at,std::string::npos);
  text.insert(at+7,"  sensor_noise:\n    model: shot_read_prnu_v1\n    enabled: true\n    pattern_seed: 7\n    realization_seed: 9\n    electrons_per_dn: 20\n    read_noise_e: 8\n    dark_current_e_per_s: 100\n    bias_dn: 4\n    prnu_fraction: 0.005\n");
  const auto parsed=Config::Parse(text);EXPECT_EQ(parsed.truth.sensor_noise.pattern_seed,7);EXPECT_TRUE(parsed.truth.sensor_noise.enabled);
  auto bad=text;bad.replace(bad.find("shot_read_prnu_v1"),16,"unknown");EXPECT_THROW(Config::Parse(bad),std::runtime_error);
  bad=text;bad.replace(bad.find("electrons_per_dn: 20"),19,"electrons_per_dn: .nan");EXPECT_THROW(Config::Parse(bad),std::runtime_error);
}
