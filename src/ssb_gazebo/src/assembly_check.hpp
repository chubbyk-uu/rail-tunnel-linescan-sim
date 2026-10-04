#pragma once
#include <gz/sim/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/JointAxis.hh>
#include "ssb_core/config.hpp"

namespace ssb_gazebo {
inline void CheckAssembly(const gz::sim::Model& model,const gz::sim::EntityComponentManager& ecm,
                          const ssb::Config& c) {
  const auto base=model.LinkByName(ecm,"base"),head=model.LinkByName(ecm,"head");
  if(base==gz::sim::kNullEntity || head==gz::sim::kNullEntity)
    throw std::runtime_error("robot assembly requires base and head links");
  const auto* bp=ecm.Component<gz::sim::components::Pose>(base);
  const auto* hp=ecm.Component<gz::sim::components::Pose>(head);
  if(!bp || !hp || std::abs(bp->Data().Pos().Z()-c.base_reference_z_m)>1e-9 ||
     std::abs(hp->Data().Pos().Z()-bp->Data().Pos().Z()-c.scan_axis_height_m-c.truth.mount.dz_m)>1e-9)
    throw std::runtime_error("robot assembly height differs between world and capture configuration");
  const auto& mount = c.truth.mount;
  const gz::math::Pose3d expected(c.truth.head_mount_x_m, mount.dy_m,
      c.scan_axis_height_m+mount.dz_m, 0., mount.tilt_y_rad, mount.tilt_z_rad);
  const auto relative = bp->Data().Inverse()*hp->Data();
  const auto rotation_error = expected.Rot().Inverse()*relative.Rot();
  if ((relative.Pos()-expected.Pos()).Length()>1e-9 ||
      rotation_error.Euler().Length()>1e-9)
    throw std::runtime_error("robot fixed scanner mount differs between world and capture configuration");
  const auto joint=model.JointByName(ecm,"scan");
  const auto* axis=ecm.Component<gz::sim::components::JointAxis>(joint);
  if (!axis || !axis->Data().XyzExpressedIn().empty() ||
      (axis->Data().Xyz()-gz::math::Vector3d(-1,0,0)).Length()>1e-9)
    throw std::runtime_error("scanner joint axis must be negative x in the mounted head frame");
}
}  // namespace ssb_gazebo
