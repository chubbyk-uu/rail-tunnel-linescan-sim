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
}  // namespace ssb_gazebo
