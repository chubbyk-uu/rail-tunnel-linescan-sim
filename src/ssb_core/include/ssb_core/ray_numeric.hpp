#pragma once

#include <cmath>
#include <cstdint>

#ifdef __CUDACC__
#define SSB_RAY_FUNCTION __host__ __device__
#else
#define SSB_RAY_FUNCTION
#endif

// Pure numeric geometry. Inputs are public measured rays and image-fitted
// corrections; this header has no scene, simulator or filesystem dependency.
namespace ssb::numeric {
struct RayResult {
  double x, q;
  double derivative[7][2];
};

SSB_RAY_FUNCTION inline bool Ray(double axis, double theta, double tangent,
    const double* correction, int fields, double radius, double height,
    bool derivatives, RayResult& result) {
  if (!std::isfinite(axis) || !std::isfinite(theta) || !std::isfinite(tangent)) return false;
  for (int field=0; field<fields; ++field)
    if (!std::isfinite(correction[field])) return false;
  const double dx=correction[0], dq=correction[1];
  const double ca=std::cos(correction[2]), sa=std::sin(correction[2]);
  const double cb=std::cos(correction[3]), sb=std::sin(correction[3]);
  theta+=dq/radius;
  const double sy=std::sin(theta), sz=std::cos(theta);
  const double ry=ca*sy-sa*sz, rz=sa*sy+ca*sz;
  const double raw_vx=cb*tangent+sb*rz, raw_vy=ry, vz=-sb*tangent+cb*rz;
  const double cy=fields==7 ? std::cos(correction[6]) : 1.;
  const double syaw=fields==7 ? std::sin(correction[6]) : 0.;
  const double vx=fields==7 ? cy*raw_vx-syaw*raw_vy : raw_vx;
  const double vy=fields==7 ? syaw*raw_vx+cy*raw_vy : raw_vy;
  const double lateral=fields>=6 ? correction[4] : 0.;
  const double vertical=fields>=6 ? correction[5] : fields==5 ? correction[4] : 0.;
  const double ox=axis+dx+height*sb*ca, oy=lateral-height*sa;
  const double oz=vertical+height*(cb*ca-1.);
  const double aa=vy*vy+vz*vz, bb=2.*(oy*vy+oz*vz);
  const double cc=oy*oy+oz*oz-radius*radius, discriminant=bb*bb-4.*aa*cc;
  if (!(aa>0.) || !(discriminant>0.)) return false;
  const double length=(-bb+std::sqrt(discriminant))/(2.*aa);
  const double py=oy+length*vy, pz=oz+length*vz;
  result.x=ox+length*vx;
  result.q=radius*std::atan2(py,pz);
  if (!std::isfinite(result.x) || !std::isfinite(result.q)) return false;
  if (!derivatives) return true;
  const double slope=py*vy+pz*vz, radial=py*py+pz*pz;
  for (int field=0; field<fields; ++field) {
    double dox=0., doy=0., doz=0., dvx=0., dvy=0., dvz=0.;
    switch (field) {
      case 0: dox=1.; break;
      case 1: dvx=-sb*ry/radius; dvy=rz/radius; dvz=-cb*ry/radius; break;
      case 2:
        dox=-height*sb*sa; doy=-height*ca; doz=-height*cb*sa;
        dvx=sb*ry; dvy=-rz; dvz=cb*ry;
        break;
      case 3: dox=height*cb*ca; doz=-height*sb*ca; dvx=vz; dvz=-raw_vx; break;
      case 4: if (fields>=6) doy=1.; else doz=1.; break;
      case 5: doz=1.; break;
      case 6: dvx=-vy; dvy=vx; break;
    }
    if (fields==7 && field!=6) {
      const double x=cy*dvx-syaw*dvy;
      dvy=syaw*dvx+cy*dvy;
      dvx=x;
    }
    const double dl=-(py*(doy+length*dvy)+pz*(doz+length*dvz))/slope;
    const double dpy=doy+dl*vy+length*dvy, dpz=doz+dl*vz+length*dvz;
    result.derivative[field][0]=dox+dl*vx+length*dvx;
    result.derivative[field][1]=radius*(pz*dpy-py*dpz)/radial;
    if (!std::isfinite(result.derivative[field][0]) ||
        !std::isfinite(result.derivative[field][1])) return false;
  }
  return true;
}

// Each block follows one side/field of the existing fixed CSR pattern. A group
// combines native pixels only when their spline rows are exactly identical.
struct JacobianBlock {
  double sign;
  int side, field, groups;
  uint32_t masks[4];
  const double* basis;
  const int64_t* inverse;
  const double* parameter_basis;
};

SSB_RAY_FUNCTION inline bool DifferenceRow(int64_t row, int fields, double radius, double height,
    const double* const* axes, const double* const* phases, const double* const* tangents,
    const double* const* weights, const double* const* corrections, double* output) {
  double points[2][2]={{0.,0.},{0.,0.}};
  for (int side=0; side<2; ++side) {
    for (int native=0; native<4; ++native) {
      const int64_t index=4*row+native;
      RayResult ray;
      const double weight=weights[side][index];
      if (!std::isfinite(weight) || !Ray(axes[side][index],phases[side][index],tangents[side][index],
          corrections[side]+fields*index,fields,radius,height,false,ray)) return false;
      points[side][0]+=ray.x*weight;
      points[side][1]+=ray.q*weight;
    }
  }
  for (int direction=0; direction<2; ++direction) {
    output[2*row+direction]=points[1][direction]-points[0][direction];
    if (!std::isfinite(output[2*row+direction])) return false;
  }
  return true;
}

SSB_RAY_FUNCTION inline bool JacobianRow(int64_t row, int fields, double radius, double height,
    const double* const* axes, const double* const* phases, const double* const* tangents,
    const double* const* weights, const double* const* corrections,
    const int64_t* starts, const JacobianBlock* blocks, int block_count,
    const int64_t* scale_positions, const double* scale_derivative,
    const double* current, const double* pitch, double* output) {
  const int64_t begin=starts[row], width=starts[row+1]-begin;
  if (begin<0 || width<=0 || !std::isfinite(current[2*row]) ||
      !std::isfinite(current[2*row+1])) return false;
  for (int64_t entry=0; entry<2*width; ++entry) output[2*begin+entry]=0.;
  RayResult rays[2][4];
  for (int side=0; side<2; ++side) {
    for (int native=0; native<4; ++native) {
      const int64_t index=4*row+native;
      if (!std::isfinite(weights[side][index]) ||
          !Ray(axes[side][index],phases[side][index],tangents[side][index],
               corrections[side]+fields*index,fields,radius,height,true,rays[side][native])) return false;
    }
  }
  const double multiplier[2]={current[2*row]/pitch[0],current[2*row+1]/pitch[1]};
  for (int block_index=0; block_index<block_count; ++block_index) {
    const auto& block=blocks[block_index];
    for (int group=0; group<block.groups; ++group) {
      double derivative[2]={0.,0.};
      for (int native=0; native<4; ++native) {
        if (!(block.masks[group] & (1U<<native))) continue;
        for (int direction=0; direction<2; ++direction)
          derivative[direction]+=rays[block.side][native].derivative[block.field][direction]*
                                 weights[block.side][4*row+native];
      }
      for (int direction=0; direction<2; ++direction) {
        const double value=derivative[direction]*(block.sign*multiplier[direction]);
        for (int spline=0; spline<4; ++spline) {
          const int64_t index=(row*block.groups+group)*4+spline;
          const int64_t position=block.inverse[index]-begin;
          const double basis=block.basis[index];
          if (position<0 || position>=width || !std::isfinite(basis)) return false;
          output[2*begin+direction*width+position]+=value*basis;
        }
      }
    }
  }
  if (scale_positions) {
    const int64_t position=scale_positions[row]-begin;
    if (position<0 || position>=width || !std::isfinite(scale_derivative[row])) return false;
    output[2*begin+position]=scale_derivative[row]*multiplier[0];
  }
  for (int64_t entry=0; entry<2*width; ++entry)
    if (!std::isfinite(output[2*begin+entry])) return false;
  return true;
}
}  // namespace ssb::numeric

#undef SSB_RAY_FUNCTION
