#pragma once
#include <cmath>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/Link.hh>
#include <sdf/Geometry.hh>
#include <sdf/Heightmap.hh>
#include <sdf/Sphere.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Box.hh>
#include <filesystem>
#include <map>
#include <nlohmann/json.hpp>
#include "ssb_core/sha256.hpp"

namespace ssb_gazebo {
// Truth options that change the physical world must be present in the loaded SDF, so a
// configuration edited without regenerating the world cannot be recorded as truth.
inline void CheckTrackAndWheels(const gz::sim::Model& car,const gz::sim::EntityComponentManager& ecm,
                                const std::string& config_text) {
  const YAML::Node truth=YAML::Load(config_text)["truth"];
  const auto irregularity=truth?truth["track_irregularity"]:YAML::Node();
  auto positive=[&](const char* key){return irregularity && irregularity[key] && irregularity[key].as<double>()>0;};
  const bool irregular=positive("chord10_max_m") || positive("cross_level_tier_m");
  int surfaces=0;
  ecm.Each<gz::sim::components::Model,gz::sim::components::Name>(
    [&](const gz::sim::Entity&,const gz::sim::components::Model*,const gz::sim::components::Name* name){
      if(name->Data().rfind("rail_surface_",0)==0)++surfaces;
      return true;});
  if(irregular!=(surfaces>0))
    throw std::runtime_error("track irregularity differs between world and capture configuration");
  const auto compliance=truth?truth["wheel_compliance"]:YAML::Node();
  int springs=0;
  for(const char* name:{"odometer_suspension","wheel_joint_1_suspension","wheel_joint_2_suspension","wheel_joint_3_suspension"}) {
    const auto joint=car.JointByName(ecm,name);
    if(joint==gz::sim::kNullEntity)continue;
    ++springs;
    if(!compliance || !compliance["stiffness_n_m"] || !compliance["damping_n_s_m"])
      throw std::runtime_error("world has sprung wheels but the configuration records no wheel compliance");
    const auto* axis=ecm.Component<gz::sim::components::JointAxis>(joint);
    const double k=compliance["stiffness_n_m"].as<double>(),c=compliance["damping_n_s_m"].as<double>();
    if(!axis || std::abs(axis->Data().SpringStiffness()-k)>1e-6*k || std::abs(axis->Data().Damping()-c)>1e-6*c)
      throw std::runtime_error("wheel compliance differs between world and capture configuration");
  }
  if(compliance && springs!=4)
    throw std::runtime_error("configuration has wheel compliance but the world lacks four sprung wheels");
}

// Loaded entities against the physical-world manifest written with the world (ssb_tools.physical_world):
// every rail heightmap model (pose, size, image hash), wheel collision radii (truth diameters) and
// spring joints. The Python launcher check also regenerates the profile from the configuration.
inline std::string PathOfUri(std::string uri) {
  if(uri.rfind("file://",0)==0) uri=uri.substr(7);
  return uri;
}
inline const sdf::Geometry* CollisionGeometry(const gz::sim::EntityComponentManager& ecm,gz::sim::Entity model,
                                              const std::string& link,const std::string& collision) {
  const gz::sim::Model m(model);const gz::sim::Link l(m.LinkByName(ecm,link));
  if(!l.Valid(ecm))return nullptr;
  const auto c=l.CollisionByName(ecm,collision);
  if(c==gz::sim::kNullEntity)return nullptr;
  const auto* g=ecm.Component<gz::sim::components::Geometry>(c);
  return g?&g->Data():nullptr;
}
// Returns the loaded heightmap image paths (for the snapshot).
inline std::vector<std::string> CheckPhysicalManifest(const gz::sim::Model& car,const gz::sim::EntityComponentManager& ecm,
                                                      const std::string& config_text,const nlohmann::json& manifest) {
  std::vector<std::string> images;
  auto fail=[](const std::string& what){throw std::runtime_error("physical world differs from its manifest: "+what);};
  if(manifest.value("schema",std::string())!="ssb.physical_manifest.v2")fail("unsupported manifest; regenerate the world");
  std::map<std::string,nlohmann::json> listed;
  for(const auto& r:manifest.at("actual").at("rails"))listed[r.at("name").get<std::string>()]=r;
  std::size_t loaded=0;
  ecm.Each<gz::sim::components::Model,gz::sim::components::Name>(
    [&](const gz::sim::Entity& e,const gz::sim::components::Model*,const gz::sim::components::Name* name){
      const std::string n=name->Data();
      if(n.rfind("rail_surface_",0)!=0)return true;
      ++loaded;
      const auto found=listed.find(n);
      if(found==listed.end())fail("unlisted rail heightmap "+n);
      const auto& r=found->second;
      const auto* pose=ecm.Component<gz::sim::components::Pose>(e);
      const auto p=pose?pose->Data():gz::math::Pose3d();
      const double got[6]={p.Pos().X(),p.Pos().Y(),p.Pos().Z(),p.Rot().Roll(),p.Rot().Pitch(),p.Rot().Yaw()};
      for(int i=0;i<6;++i)if(std::abs(got[i]-r.at("pose")[i].get<double>())>1e-9)fail("pose of "+n);
      const auto* g=CollisionGeometry(ecm,e,"top","rail_top");
      if(!g||!g->HeightmapShape())fail("heightmap geometry of "+n);
      const auto size=g->HeightmapShape()->Size();
      const double want[3]={r.at("size")[0],r.at("size")[1],r.at("size")[2]};
      if(std::abs(size.X()-want[0])>1e-9||std::abs(size.Y()-want[1])>1e-9||std::abs(size.Z()-want[2])>1e-9)fail("size of "+n);
      const std::string path=PathOfUri(g->HeightmapShape()->Uri());
      if(!std::filesystem::exists(path)||ssb::Sha256File(path)!=r.at("sha256").get<std::string>())fail("image of "+n);
      images.push_back(path);
      return true;});
  if(loaded!=listed.size())fail("rail heightmaps missing ("+std::to_string(loaded)+" of "+std::to_string(listed.size())+")");
  // Both the flat running surfaces and the lowered guide faces must match the manifest.
  gz::sim::Entity track=gz::sim::kNullEntity;
  ecm.Each<gz::sim::components::Model,gz::sim::components::Name>(
    [&](const gz::sim::Entity& e,const auto*,const auto* name){
      if(name->Data()=="track") {
        if(track!=gz::sim::kNullEntity)fail("duplicate track model");
        track=e;
      }
      return true;
    });
  if(track==gz::sim::kNullEntity)fail("missing track model");
  const auto rails=gz::sim::Model(track).LinkByName(ecm,"rails");
  auto pose_matches=[&](gz::sim::Entity entity,const nlohmann::json& want,const std::string& label){
    const auto* component=ecm.Component<gz::sim::components::Pose>(entity);
    const auto p=component?component->Data():gz::math::Pose3d();
    const double values[6]={p.Pos().X(),p.Pos().Y(),p.Pos().Z(),p.Rot().Roll(),p.Rot().Pitch(),p.Rot().Yaw()};
    for(int i=0;i<6;++i)if(std::abs(values[i]-want.at(i).get<double>())>1e-9)fail("pose of "+label);
  };
  if(rails==gz::sim::kNullEntity)fail("missing track rails link");
  std::size_t boxes=0;
  ecm.Each<gz::sim::components::Collision,gz::sim::components::ParentEntity>(
    [&](const gz::sim::Entity&,const auto*,const auto* parent){if(parent->Data()==rails)++boxes;return true;});
  const auto& rail_boxes=manifest.at("actual").at("rail_boxes");
  if(boxes!=rail_boxes.size()||boxes!=2)fail("rail collision box set");
  for(const auto& box:rail_boxes) {
    const auto name=box.at("name").get<std::string>();
    const auto collision=gz::sim::Link(rails).CollisionByName(ecm,name);
    if(collision==gz::sim::kNullEntity)fail("missing rail box "+name);
    const auto* g=CollisionGeometry(ecm,track,"rails",name);
    if(!g||!g->BoxShape()||box.at("shape")!="box")fail("geometry of rail box "+name);
    const auto size=g->BoxShape()->Size();
    const auto& want=box.at("size");
    if(std::abs(size.X()-want.at(0).get<double>())>1e-9||std::abs(size.Y()-want.at(1).get<double>())>1e-9||
       std::abs(size.Z()-want.at(2).get<double>())>1e-9)fail("size of rail box "+name);
    pose_matches(track,box.at("model_pose"),"track model");
    pose_matches(rails,box.at("link_pose"),"rails link");
    pose_matches(collision,box.at("pose"),name);
  }
  // Wheel collision radii: the truth diameters, not what the world happens to contain.
  const YAML::Node truth=YAML::Load(config_text)["truth"];
  auto radius=[&](const std::string& link,bool sphere)->double{
    const auto* g=CollisionGeometry(ecm,car.Entity(),link,"tread_contact");
    if(!g)fail("collision of "+link);
    if(sphere){if(!g->SphereShape())fail(link+" is not a sphere");return g->SphereShape()->Radius();}
    if(!g->CylinderShape())fail(link+" is not a cylinder");
    return g->CylinderShape()->Radius();};
  for(const char* link:{"odometer_wheel","wheel_1","wheel_2","wheel_3"})
    if(std::abs(radius(link,false)-truth["wheel_diameter_m"].as<double>()/2)>1e-12)fail("running wheel radius "+std::string(link));
  for(const char* side:{"left","right"})
    if(std::abs(radius(std::string("measure_")+side+"_wheel",true)-truth[std::string("odo_")+side+"_diameter_m"].as<double>()/2)>1e-12)
      fail(std::string("measuring wheel radius (")+side+"): world and truth diameter differ; regenerate the world");
  // Measuring-wheel slides as recorded (preload, rate, damping, travel).
  for(const auto& [name,j]:manifest.at("expected").at("joints").items()) {
    if(name.size()<6||name.substr(name.size()-6)!="_slide")continue;
    const auto joint=car.JointByName(ecm,name);
    const auto* axis=joint==gz::sim::kNullEntity?nullptr:ecm.Component<gz::sim::components::JointAxis>(joint);
    if(!axis)fail("missing "+name);
    const auto& a=axis->Data();
    auto near=[](double x,double y){return std::abs(x-y)<=1e-9*std::max(1.,std::abs(y));};
    if(!near(a.SpringStiffness(),j.at("stiffness"))||!near(a.Damping(),j.at("damping"))||!near(a.SpringReference(),j.at("reference"))||
       !near(a.Lower(),j.at("lower"))||!near(a.Upper(),j.at("upper")))fail("spring of "+name);
  }
  return images;
}

// Archive the physical inputs with the run, so later validation does not depend on the demo folder.
inline void SnapshotPhysical(const std::filesystem::path& world,const std::filesystem::path& config,
                             const std::vector<std::string>& images,const std::filesystem::path& destination) {
  namespace fs=std::filesystem;
  fs::create_directories(destination);
  std::map<std::string,fs::path> copied;
  auto copy=[&](const fs::path& from,bool required=true){
    if(!fs::is_regular_file(from)) {
      if(required)throw std::runtime_error("missing physical snapshot input: "+from.string());
      return;
    }
    const auto key=from.filename().string();
    const auto source=fs::canonical(from);
    if(copied.count(key)&&copied.at(key)!=source)
      throw std::runtime_error("physical snapshot basename collision: "+key);
    copied[key]=source;
    fs::copy_file(from,destination/from.filename(),fs::copy_options::overwrite_existing);
  };
  copy(world.parent_path()/"physical_manifest.json");copy(world);copy(config.parent_path()/"spec.yaml");copy(config);
  for(const auto& image:images)copy(image);
  if(!images.empty()) {
    const fs::path track=fs::path(images.front()).parent_path();
    copy(track/"rail_irregularity.json",false);copy(track/"rail_profile.npz",false);
  }
}
}  // namespace ssb_gazebo
