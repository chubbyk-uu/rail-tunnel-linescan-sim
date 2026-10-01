#pragma once
#include <gz/sim/Model.hh>
#include <gz/sim/components/Pose.hh>
#include "ssb_core/config.hpp"

namespace ssb_gazebo {
inline void CheckAssembly(const gz::sim::Model& model,const gz::sim::EntityComponentManager& ecm,
                          const ssb::Config& c) {
  const auto base=model.LinkByName(ecm,"base"),head=model.LinkByName(ecm,"head");
  if(base==gz::sim::kNullEntity || head==gz::sim::kNullEntity) return; // Stage A ideal fixture
  const auto* bp=ecm.Component<gz::sim::components::Pose>(base);
  const auto* hp=ecm.Component<gz::sim::components::Pose>(head);
  if(!bp || !hp || std::abs(bp->Data().Pos().Z()-c.base_reference_z_m)>1e-9 ||
     std::abs(hp->Data().Pos().Z()-bp->Data().Pos().Z()-c.scan_axis_height_m)>1e-9)
    throw std::runtime_error("robot assembly height differs between world and capture configuration");
}
}  // namespace ssb_gazebo
