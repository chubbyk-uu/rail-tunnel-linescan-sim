#include <gtest/gtest.h>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include "control_math.hpp"
#include "assembly_check.hpp"

TEST(Control, EulerRatesMatchIndependentMatrixDerivative) {
  auto rotation=[](double r,double p,double y) {
    const double cr=cos(r),sr=sin(r),cp=cos(p),sp=sin(p),cy=cos(y),sy=sin(y);
    return std::array<std::array<double,3>,3>{{{cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr},
      {sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr},{-sp,cp*sr,cp*cr}}};
  };
  for(double pitch:{-.7,0.,.4,1.1}) for(double yaw:{-.8,0.,2.}) {
    const double roll=.3,eps=1e-6;
    const std::array<double,3> wanted{.17,-.23,.31};
    auto r=rotation(roll,pitch,yaw),plus=rotation(roll+eps*wanted[0],pitch+eps*wanted[1],yaw+eps*wanted[2]),
         minus=rotation(roll-eps*wanted[0],pitch-eps*wanted[1],yaw-eps*wanted[2]);
    double skew[3][3]{};
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) for(int k=0;k<3;++k)
      skew[i][j]+=(plus[i][k]-minus[i][k])/(2*eps)*r[j][k];
    auto got=ssb_gazebo::EulerRates(pitch,yaw,{skew[2][1],skew[0][2],skew[1][0]});
    for(int i=0;i<3;++i) EXPECT_NEAR(got[i],wanted[i],1e-9);
  }
}

TEST(Control, ServoFollowsBiasedQuantizedOdometryAndStops) {
  ssb_gazebo::ScanServo servo;
  const double dt=.001,start=M_PI,pitch=.6,quantum=M_PI*.22/10000;
  double angle=start;
  for(int i=0;i<12000;++i) {
    const double t=i*dt,s=std::clamp(t-9.,0.,1.);
    const double travel=t<1 ? .11*(t-std::sin(M_PI*t)/M_PI) :
                        t<9 ? .11+.22*(t-1) : 1.87+.11*(s+std::sin(M_PI*s)/M_PI);
    const double distance=std::floor(travel/quantum)*quantum;
    const double rate=servo.Update(distance,angle,dt,start,pitch);
    EXPECT_GE(rate,0);EXPECT_LE(rate,3.3);
    EXPECT_NEAR(servo.Target(),start+2*M_PI*distance/pitch,1e-12);
    angle+=rate*dt;
  }
  EXPECT_NEAR(angle,servo.Target(),.012); // same tracking limit as contact acceptance
  EXPECT_LT(servo.Update(1.98,angle,dt,start,pitch),1e-6);
  EXPECT_GT(angle,start+2*M_PI*1.8/pitch); // responds to measured diameter bias, not true travel
}

TEST(Assembly, ReadsActualLinkPosesAndRejectsMismatch) {
  gz::sim::EntityComponentManager ecm;
  auto model=ecm.CreateEntity();ecm.CreateComponent(model,gz::sim::components::Model());
  auto link=[&](const char* name,double z) {
    auto e=ecm.CreateEntity();ecm.CreateComponent(e,gz::sim::components::Link());
    ecm.CreateComponent(e,gz::sim::components::Name(name));
    ecm.CreateComponent(e,gz::sim::components::ParentEntity(model));
    ecm.CreateComponent(e,gz::sim::components::Pose(gz::math::Pose3d(0,0,z,0,0,0)));
  };
  link("base",.37);link("head",2.015);
  ssb::Config c;c.base_reference_z_m=.37;c.scan_axis_height_m=1.645;
  EXPECT_NO_THROW(ssb_gazebo::CheckAssembly(gz::sim::Model(model),ecm,c));
  c.base_reference_z_m=.3;
  EXPECT_THROW(ssb_gazebo::CheckAssembly(gz::sim::Model(model),ecm,c),std::runtime_error);
  c.base_reference_z_m=.37;c.scan_axis_height_m=1.715;
  EXPECT_THROW(ssb_gazebo::CheckAssembly(gz::sim::Model(model),ecm,c),std::runtime_error);
}

#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointAxis.hh>
#include <sdf/JointAxis.hh>
#include "world_check.hpp"

namespace {
gz::sim::Entity AddModel(gz::sim::EntityComponentManager& ecm,const std::string& name) {
  auto e=ecm.CreateEntity();ecm.CreateComponent(e,gz::sim::components::Model());
  ecm.CreateComponent(e,gz::sim::components::Name(name));return e;
}
void AddSpring(gz::sim::EntityComponentManager& ecm,gz::sim::Entity car,const std::string& name,double k,double c) {
  auto e=ecm.CreateEntity();ecm.CreateComponent(e,gz::sim::components::Joint());
  ecm.CreateComponent(e,gz::sim::components::Name(name));ecm.CreateComponent(e,gz::sim::components::ParentEntity(car));
  sdf::JointAxis axis;axis.SetSpringStiffness(k);axis.SetDamping(c);
  ecm.CreateComponent(e,gz::sim::components::JointAxis(axis));
}
const char* kSprings[]={"odometer_suspension","wheel_joint_1_suspension","wheel_joint_2_suspension","wheel_joint_3_suspension"};
}

TEST(WorldCheck, TrackIrregularityMustMatchWorldHeightmaps) {
  gz::sim::EntityComponentManager ecm;auto car=AddModel(ecm,"scan_car");
  const std::string flat="truth: {wheel_diameter_m: 0.2}\n";
  const std::string rough="truth: {track_irregularity: {chord10_max_m: 0.002, seed: 1}}\n";
  EXPECT_NO_THROW(ssb_gazebo::CheckTrackAndWheels(gz::sim::Model(car),ecm,flat));
  EXPECT_THROW(ssb_gazebo::CheckTrackAndWheels(gz::sim::Model(car),ecm,rough),std::runtime_error);
  AddModel(ecm,"rail_surface_left_00");
  EXPECT_NO_THROW(ssb_gazebo::CheckTrackAndWheels(gz::sim::Model(car),ecm,rough));
  EXPECT_THROW(ssb_gazebo::CheckTrackAndWheels(gz::sim::Model(car),ecm,flat),std::runtime_error);
  // A cross-level-only track is irregular too.
  EXPECT_NO_THROW(ssb_gazebo::CheckTrackAndWheels(gz::sim::Model(car),ecm,
    "truth: {track_irregularity: {chord10_max_m: 0, cross_level_tier_m: 0.002, seed: 1}}\n"));
}

TEST(WorldCheck, WheelComplianceMustMatchSdfSprings) {
  gz::sim::EntityComponentManager ecm;auto car=AddModel(ecm,"scan_car");
  const std::string sprung="truth: {wheel_compliance: {static_deflection_m: 0.0002, stiffness_n_m: 2.9e6, damping_n_s_m: 3300}}\n";
  const std::string rigid="truth: {wheel_diameter_m: 0.2}\n";
  EXPECT_THROW(ssb_gazebo::CheckTrackAndWheels(gz::sim::Model(car),ecm,sprung),std::runtime_error);  // no springs
  for(int i=0;i<3;++i)AddSpring(ecm,car,kSprings[i],2.9e6,3300);
  EXPECT_THROW(ssb_gazebo::CheckTrackAndWheels(gz::sim::Model(car),ecm,sprung),std::runtime_error);  // three of four
  AddSpring(ecm,car,kSprings[3],2.9e6*1.01,3300);
  EXPECT_THROW(ssb_gazebo::CheckTrackAndWheels(gz::sim::Model(car),ecm,sprung),std::runtime_error);  // stiffness differs
  EXPECT_THROW(ssb_gazebo::CheckTrackAndWheels(gz::sim::Model(car),ecm,rigid),std::runtime_error);   // springs, no config
}

TEST(WorldCheck, MatchingSprungWorldIsAccepted) {
  gz::sim::EntityComponentManager ecm;auto car=AddModel(ecm,"scan_car");
  for(auto* n:kSprings)AddSpring(ecm,car,n,2.9e6,3300);
  EXPECT_NO_THROW(ssb_gazebo::CheckTrackAndWheels(gz::sim::Model(car),ecm,
    "truth: {wheel_compliance: {static_deflection_m: 0.0002, stiffness_n_m: 2.9e6, damping_n_s_m: 3300}}\n"));
}
