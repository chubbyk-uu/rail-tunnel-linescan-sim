#include "ssb_core/ray_numeric.hpp"

namespace {
bool Geometry(int fields, double radius, double height) {
  return fields>=4 && fields<=7 && std::isfinite(radius) && radius>0. &&
         std::isfinite(height) && height>0.;
}
}

extern "C" int ssb_ray_abi() { return 2; }

extern "C" int ssb_ray_hits(int64_t count, int fields, double radius, double height,
    const double* axis, const double* phase, const double* tangent, const double* correction,
    int derivatives, double* output) {
  if (count<0 || !Geometry(fields,radius,height) || !axis || !phase || !tangent ||
      !correction || !output || (derivatives!=0 && derivatives!=1)) return 1;
  for (int64_t row=0; row<count; ++row) {
    ssb::numeric::RayResult result;
    if (!ssb::numeric::Ray(axis[row],phase[row],tangent[row],correction+fields*row,
                           fields,radius,height,derivatives,result)) return 2;
    if (derivatives) {
      for (int field=0; field<fields; ++field)
        for (int direction=0; direction<2; ++direction)
          output[(row*fields+field)*2+direction]=result.derivative[field][direction];
    } else {
      output[2*row]=result.x; output[2*row+1]=result.q;
    }
  }
  return 0;
}

extern "C" int ssb_ray_jacobian(int64_t first, int64_t last, int fields, double radius, double height,
    const double* const* axes, const double* const* phases, const double* const* tangents,
    const double* const* weights, const double* const* corrections,
    const int64_t* starts, const ssb::numeric::JacobianBlock* blocks, int block_count,
    const int64_t* scale_positions, const double* scale_derivative,
    const double* current, const double* pitch, double* output) {
  if (first<0 || last<first || !Geometry(fields,radius,height) || !axes || !phases ||
      !tangents || !weights || !corrections || !starts || !blocks || block_count<1 ||
      !current || !pitch || !output || !(pitch[0]>0.) || !(pitch[1]>0.) ||
      !std::isfinite(pitch[0]) || !std::isfinite(pitch[1]) ||
      (scale_positions && !scale_derivative)) return 1;
  for (int side=0; side<2; ++side)
    if (!axes[side] || !phases[side] || !tangents[side] || !weights[side] || !corrections[side]) return 1;
  for (int index=0; index<block_count; ++index) {
    const auto& block=blocks[index];
    if (block.side<0 || block.side>1 || block.field<0 || block.field>=fields ||
        block.groups<1 || block.groups>4 || !block.basis || !block.inverse ||
        (block.sign!=1. && block.sign!=-1.)) return 1;
    for (int group=0; group<block.groups; ++group)
      if (!block.masks[group] || block.masks[group]>15) return 1;
  }
  for (int64_t row=first; row<last; ++row)
    if (!ssb::numeric::JacobianRow(row,fields,radius,height,axes,phases,tangents,weights,corrections,
        starts,blocks,block_count,scale_positions,scale_derivative,current,pitch,output)) return 2;
  return 0;
}

extern "C" int ssb_ray_difference(int64_t first, int64_t last, int fields, double radius, double height,
    const double* const* axes, const double* const* phases, const double* const* tangents,
    const double* const* weights, const double* const* corrections, double* output) {
  if (first<0 || last<first || !Geometry(fields,radius,height) || !axes || !phases ||
      !tangents || !weights || !corrections || !output) return 1;
  for (int side=0; side<2; ++side)
    if (!axes[side] || !phases[side] || !tangents[side] || !weights[side] || !corrections[side]) return 1;
  for (int64_t row=first; row<last; ++row)
    if (!ssb::numeric::DifferenceRow(row,fields,radius,height,axes,phases,tangents,weights,corrections,output)) return 2;
  return 0;
}
