#include <gtest/gtest.h>
#include <optional>
#include <vector>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include "control_math.hpp"
#include "assembly_check.hpp"

TEST(Control, ZeroTargetBrakesBothDirectionsWithBoundedTorque) {
  for(double rate:{-10.,-2.,-.1,.1,2.,10.}) {
    const double torque=ssb_gazebo::DriveTorque(0.,rate);
    EXPECT_LT(torque*rate,0.);
    EXPECT_LE(std::abs(torque),8.);
  }
  EXPECT_DOUBLE_EQ(ssb_gazebo::DriveTorque(0.,0.),0.);
}

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

#include <gz/sim/components/Joint.hh>
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
  auto joint=ecm.CreateEntity();ecm.CreateComponent(joint,gz::sim::components::Joint());
  ecm.CreateComponent(joint,gz::sim::components::Name("scan"));
  ecm.CreateComponent(joint,gz::sim::components::ParentEntity(model));
  sdf::JointAxis axis;axis.SetXyz(gz::math::Vector3d(-1,0,0));
  ecm.CreateComponent(joint,gz::sim::components::JointAxis(axis));
  ssb::Config c;c.base_reference_z_m=.37;c.scan_axis_height_m=1.645;
  EXPECT_NO_THROW(ssb_gazebo::CheckAssembly(gz::sim::Model(model),ecm,c));
  c.base_reference_z_m=.3;
  EXPECT_THROW(ssb_gazebo::CheckAssembly(gz::sim::Model(model),ecm,c),std::runtime_error);
  c.base_reference_z_m=.37;c.scan_axis_height_m=1.715;
  EXPECT_THROW(ssb_gazebo::CheckAssembly(gz::sim::Model(model),ecm,c),std::runtime_error);
}

TEST(Assembly, FixedOffsetTiltAndShaftFrameAreChecked) {
  gz::sim::EntityComponentManager ecm;
  auto model=ecm.CreateEntity();ecm.CreateComponent(model,gz::sim::components::Model());
  ssb::Config c;c.base_reference_z_m=.3;c.scan_axis_height_m=1.715;
  c.truth.head_mount_x_m=.007;c.truth.mount.dy_m=.02;c.truth.mount.dz_m=-.013;
  c.truth.mount.tilt_y_rad=.001;c.truth.mount.tilt_z_rad=-.0015;
  for (const char* name : {"base","head"}) {
    auto link=ecm.CreateEntity();ecm.CreateComponent(link,gz::sim::components::Link());
    ecm.CreateComponent(link,gz::sim::components::Name(name));
    ecm.CreateComponent(link,gz::sim::components::ParentEntity(model));
    const auto pose=std::string(name)=="base" ? gz::math::Pose3d(0,0,.3,0,0,0) :
      gz::math::Pose3d(.007,.02,2.002,0,.001,-.0015);
    ecm.CreateComponent(link,gz::sim::components::Pose(pose));
  }
  auto joint=ecm.CreateEntity();ecm.CreateComponent(joint,gz::sim::components::Joint());
  ecm.CreateComponent(joint,gz::sim::components::Name("scan"));
  ecm.CreateComponent(joint,gz::sim::components::ParentEntity(model));
  sdf::JointAxis axis;axis.SetXyz(gz::math::Vector3d(-1,0,0));
  ecm.CreateComponent(joint,gz::sim::components::JointAxis(axis));
  EXPECT_NO_THROW(ssb_gazebo::CheckAssembly(gz::sim::Model(model),ecm,c));
  c.truth.mount.tilt_z_rad=0;
  EXPECT_THROW(ssb_gazebo::CheckAssembly(gz::sim::Model(model),ecm,c),std::runtime_error);
  c.truth.mount.tilt_z_rad=-.0015;
  axis.SetXyzExpressedIn("__model__");
  ecm.Component<gz::sim::components::JointAxis>(joint)->Data()=axis;
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

#include <fstream>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Pose.hh>
#include <sdf/Geometry.hh>
#include <sdf/Heightmap.hh>
#include <sdf/Sphere.hh>
#include <sdf/Cylinder.hh>

namespace {
gz::sim::Entity AddChild(gz::sim::EntityComponentManager& ecm,gz::sim::Entity parent,const std::string& name,bool link) {
  auto e=ecm.CreateEntity();
  if(link)ecm.CreateComponent(e,gz::sim::components::Link());else ecm.CreateComponent(e,gz::sim::components::Collision());
  ecm.CreateComponent(e,gz::sim::components::Name(name));ecm.CreateComponent(e,gz::sim::components::ParentEntity(parent));
  return e;
}
void AddShape(gz::sim::EntityComponentManager& ecm,gz::sim::Entity model,const std::string& link,
              const std::string& collision,const sdf::Geometry& g) {
  auto c=AddChild(ecm,AddChild(ecm,model,link,true),collision,false);
  ecm.CreateComponent(c,gz::sim::components::Geometry(g));
}
// A manifest and matching loaded entities: one rail heightmap, six wheels, two measuring slides.
struct PhysicalFixture {
  gz::sim::EntityComponentManager ecm;gz::sim::Entity car,rail,track,rails,left_box;
  nlohmann::json manifest;std::string config;
  std::filesystem::path image=std::filesystem::temp_directory_path()/"ssb_test_rail_top.png";
  PhysicalFixture() {
    std::ofstream(image)<<"heightmap bytes";
    car=AddModel(ecm,"scan_car");
    for(const char* n:{"odometer_wheel","wheel_1","wheel_2","wheel_3"}) {
      sdf::Geometry g;g.SetType(sdf::GeometryType::CYLINDER);sdf::Cylinder c;c.SetRadius(.1);g.SetCylinderShape(c);
      AddShape(ecm,car,n,"tread_contact",g);}
    for(const char* side:{"left","right"}) {
      sdf::Geometry g;g.SetType(sdf::GeometryType::SPHERE);sdf::Sphere s;s.SetRadius(.04);g.SetSphereShape(s);
      AddShape(ecm,car,std::string("measure_")+side+"_wheel","tread_contact",g);
      auto j=ecm.CreateEntity();ecm.CreateComponent(j,gz::sim::components::Joint());
      ecm.CreateComponent(j,gz::sim::components::Name(std::string("measure_")+side+"_slide"));
      ecm.CreateComponent(j,gz::sim::components::ParentEntity(car));
      sdf::JointAxis axis;axis.SetSpringStiffness(2000);axis.SetDamping(20);axis.SetSpringReference(-.015);
      axis.SetLower(-.012);axis.SetUpper(.012);ecm.CreateComponent(j,gz::sim::components::JointAxis(axis));
      manifest["expected"]["joints"][std::string("measure_")+side+"_slide"]=
        {{"stiffness",2000.},{"damping",20.},{"reference",-.015},{"lower",-.012},{"upper",.012}};
    }
    rail=AddModel(ecm,"rail_surface_left_00");
    ecm.CreateComponent(rail,gz::sim::components::Pose(gz::math::Pose3d(1.5,.754,-.001,0,0,0)));
    sdf::Geometry g;g.SetType(sdf::GeometryType::HEIGHTMAP);sdf::Heightmap h;
    h.SetUri("file://"+image.string());h.SetSize({3.,.073,.002});g.SetHeightmapShape(h);
    AddShape(ecm,rail,"top","rail_top",g);
    manifest["schema"]="ssb.physical_manifest.v2";
    manifest["actual"]["rails"]=nlohmann::json::array({{{"name","rail_surface_left_00"},{"pose",{1.5,.754,-.001,0,0,0}},
      {"size",{3.,.073,.002}},{"sha256",ssb::Sha256File(image)}}});
    track=AddModel(ecm,"track");rails=AddChild(ecm,track,"rails",true);
    manifest["actual"]["rail_boxes"]=nlohmann::json::array();
    for(const char* side:{"left","right"}) {
      const auto name=std::string(side)+"_head";
      const double y=std::string(side)=="left"?.754:-.754;
      const auto collision=AddChild(ecm,rails,name,false);
      if(std::string(side)=="left")left_box=collision;
      sdf::Geometry box;box.SetType(sdf::GeometryType::BOX);sdf::Box shape;shape.SetSize({3.,.073,.036});box.SetBoxShape(shape);
      ecm.CreateComponent(collision,gz::sim::components::Geometry(box));
      ecm.CreateComponent(collision,gz::sim::components::Pose(gz::math::Pose3d(1.5,y,-.020,0,0,0)));
      manifest["actual"]["rail_boxes"].push_back({{"name",name},{"shape","box"},{"model_pose",{0,0,0,0,0,0}},
        {"link_pose",{0,0,0,0,0,0}},{"pose",{1.5,y,-.020,0,0,0}},{"size",{3.,.073,.036}}});
    }
    config="truth: {wheel_diameter_m: 0.2, odo_left_diameter_m: 0.08, odo_right_diameter_m: 0.08}\n";
  }
  void Check(){ssb_gazebo::CheckPhysicalManifest(gz::sim::Model(car),ecm,config,manifest);}
};
}

TEST(WorldCheck, LoadedPhysicalWorldMatchesManifest) {
  PhysicalFixture f;
  EXPECT_NO_THROW(f.Check());
}

TEST(WorldCheck, MovedOrMissingRailHeightmapIsRejected) {
  PhysicalFixture moved;moved.ecm.Component<gz::sim::components::Pose>(moved.rail)->Data().Pos().Y(.854);
  EXPECT_THROW(moved.Check(),std::runtime_error);
  PhysicalFixture missing;missing.manifest["actual"]["rails"].push_back(missing.manifest["actual"]["rails"][0]);
  missing.manifest["actual"]["rails"][1]["name"]="rail_surface_right_00";
  EXPECT_THROW(missing.Check(),std::runtime_error);
  PhysicalFixture image;image.manifest["actual"]["rails"][0]["sha256"]="0";
  EXPECT_THROW(image.Check(),std::runtime_error);
}

TEST(WorldCheck, WheelDiameterOrSlideSpringMismatchIsRejected) {
  PhysicalFixture wheel;wheel.config="truth: {wheel_diameter_m: 0.2, odo_left_diameter_m: 0.081, odo_right_diameter_m: 0.08}\n";
  EXPECT_THROW(wheel.Check(),std::runtime_error);
  PhysicalFixture spring;spring.manifest["expected"]["joints"]["measure_right_slide"]["stiffness"]=2500.;
  EXPECT_THROW(spring.Check(),std::runtime_error);
}

TEST(WorldCheck, LoadedRailBoxesCheckPosesSizesAndCompleteSet) {
  PhysicalFixture moved;
  moved.ecm.Component<gz::sim::components::Pose>(moved.left_box)->Data().Pos().Z(.08);
  EXPECT_THROW(moved.Check(),std::runtime_error);
  PhysicalFixture width;
  auto& geometry=width.ecm.Component<gz::sim::components::Geometry>(width.left_box)->Data();
  sdf::Box shape;shape.SetSize({3.,.08,.036});geometry.SetBoxShape(shape);
  EXPECT_THROW(width.Check(),std::runtime_error);
  PhysicalFixture model;
  model.ecm.CreateComponent(model.track,gz::sim::components::Pose(gz::math::Pose3d(0,0,.1,0,0,0)));
  EXPECT_THROW(model.Check(),std::runtime_error);
  PhysicalFixture link;
  link.ecm.CreateComponent(link.rails,gz::sim::components::Pose(gz::math::Pose3d(0,0,0,0,0,.1)));
  EXPECT_THROW(link.Check(),std::runtime_error);
  PhysicalFixture missing;missing.ecm.RemoveComponent<gz::sim::components::Collision>(missing.left_box);
  EXPECT_THROW(missing.Check(),std::runtime_error);
}

TEST(Control, MissingOrEmptyJointVectorsAreRejected) {
  std::optional<std::vector<double>> good(std::vector<double>{1.});
  std::optional<std::vector<double>> missing,empty(std::vector<double>{});
  EXPECT_TRUE(ssb_gazebo::HasValues(good,good));
  EXPECT_FALSE(ssb_gazebo::HasValues(good,missing));
  EXPECT_FALSE(ssb_gazebo::HasValues(empty,good));
}

TEST(Control, DistanceCompletionRequiresTargetRatesAndContinuousHold) {
  ssb::Config::DistanceStop d;d.target_m=3.;d.timeout_s=40.;
  ssb_gazebo::DistanceController controller;
  // Past the former 17 s motion deadline, a lagging encoder still commands motion.
  EXPECT_DOUBLE_EQ(controller.Update(d,.2,20.,2.,.2,.001,false),.2);
  EXPECT_FALSE(controller.Complete());
  for(int i=0;i<600;++i) controller.Update(d,.2,25.,3.,.01,.001,true);
  EXPECT_FALSE(controller.Complete());
  for(int i=0;i<400;++i) controller.Update(d,.2,25.,3.,0.,.001,true);
  controller.Update(d,.2,25.,3.,0.,.001,false); // unsettled scanner resets hold
  for(int i=0;i<400;++i) controller.Update(d,.2,25.,3.,0.,.001,true);
  EXPECT_FALSE(controller.Complete());
  for(int i=0;i<101;++i) controller.Update(d,.2,25.,3.,0.,.001,true);
  EXPECT_TRUE(controller.Complete());
}

TEST(Control, BiasedQuantizedWheelDistanceSetsActualStoppingAndPitch) {
  for(double diameter:{.079,.080,.081}) {
    ssb::Config::DistanceStop d;d.target_m=3.;d.timeout_s=40.;
    ssb_gazebo::DistanceController controller;
    double travel=0.,speed=0.;
    const double dt=.001,quantum=M_PI*.08/10000.;
    for(int i=0;i<40000 && !controller.Complete();++i) {
      const double estimate=std::floor(travel*.08/diameter/quantum)*quantum;
      const double command=controller.Update(d,.2,i*dt,estimate,speed*.08/diameter,dt,true);
      speed+=(command-speed)*dt/(.01+dt);
      travel+=speed*dt;
    }
    ASSERT_TRUE(controller.Complete());
    EXPECT_NEAR(travel,3.*diameter/.08,.00015);
    EXPECT_NEAR(travel/5.,.6*diameter/.08,.00003);
  }
}

TEST(Control, OvershootCannotBeDeclaredSuccessfulParking) {
  ssb::Config::DistanceStop d;d.target_m=3.;d.timeout_s=40.;
  ssb_gazebo::DistanceController controller;
  for(int i=0;i<1000;++i) {
    EXPECT_DOUBLE_EQ(controller.Update(d,.2,30.,3.002,0.,.001,true),0.);
  }
  EXPECT_FALSE(controller.Complete());
}

TEST(Control, DriveIntegralHoldsAgainstGradeAndDoesNotWindUp) {
  ssb_gazebo::DriveController controller;
  double rate=0.;
  for(int i=0;i<10000;++i) {
    const double torque=controller.Update(0.,rate,.001);
    rate+=(torque+.12)*.001/.15; // independent wheel inertia + constant grade torque
  }
  EXPECT_NEAR(rate,0.,1e-8);
  ssb_gazebo::DriveController saturated;
  for(int i=0;i<10000;++i) EXPECT_DOUBLE_EQ(saturated.Update(100.,0.,.001),8.);
  EXPECT_DOUBLE_EQ(saturated.Update(0.,0.,.001),0.);
}

TEST(Assembly, MissingRequiredLinksRejectTheModel) {
  gz::sim::EntityComponentManager ecm;
  auto model=ecm.CreateEntity();ecm.CreateComponent(model,gz::sim::components::Model());
  ssb::Config c;
  EXPECT_THROW(ssb_gazebo::CheckAssembly(gz::sim::Model(model),ecm,c),std::runtime_error);
  auto base=ecm.CreateEntity();ecm.CreateComponent(base,gz::sim::components::Link());
  ecm.CreateComponent(base,gz::sim::components::Name("base"));
  ecm.CreateComponent(base,gz::sim::components::ParentEntity(model));
  EXPECT_THROW(ssb_gazebo::CheckAssembly(gz::sim::Model(model),ecm,c),std::runtime_error);
}
