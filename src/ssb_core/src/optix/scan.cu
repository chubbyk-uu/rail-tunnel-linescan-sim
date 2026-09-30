#include <optix.h>

#include "launch_params.h"
#include "ssb_core/texture.hpp"
#include "ssb_core/crack_integral.hpp"

extern "C" {
__constant__ LaunchParams params;
}

namespace {
__device__ unsigned Lo(double v) { return static_cast<unsigned>(__double_as_longlong(v) & 0xffffffffull); }
__device__ unsigned Hi(double v) { return static_cast<unsigned>(static_cast<unsigned long long>(__double_as_longlong(v)) >> 32); }
__device__ double Join(unsigned lo, unsigned hi) {
  return __longlong_as_double(static_cast<long long>((static_cast<unsigned long long>(hi) << 32) | lo));
}
}  // namespace

// Analytic cylinder about (y, z) = (0, axis_z); the ray starts inside it.
extern "C" __global__ void __intersection__tunnel() {
  const float3 o = optixGetObjectRayOrigin(), d = optixGetObjectRayDirection();
  const double oy = o.y, oz = o.z - params.axis_z, dy = d.y, dz = d.z;
  const double a = dy * dy + dz * dz;
  if (a <= 0) return;
  const double b = 2 * (oy * dy + oz * dz), c = oy * oy + oz * oz - params.radius * params.radius;
  const double disc = b * b - 4 * a * c;
  if (disc < 0) return;
  const double q = -0.5 * (b + copysign(sqrt(disc), b));
  const double r1 = q / a, r2 = q != 0 ? c / q : r1;
  const double t = fmax(r1, r2);
  if (t >= optixGetRayTmin() && t <= optixGetRayTmax())
    optixReportIntersection(static_cast<float>(t), 0, Lo(t), Hi(t));
}

extern "C" __global__ void __closesthit__tunnel() {
  optixSetPayload_0(optixGetAttribute_0());
  optixSetPayload_1(optixGetAttribute_1());
  optixSetPayload_2(1u);
}

extern "C" __global__ void __miss__primary() { optixSetPayload_2(0u); }

__device__ void StageBScan(unsigned u, unsigned r);

extern "C" __global__ void __raygen__scan() {
  const uint3 idx = optixGetLaunchIndex();
  const unsigned u = idx.x, r = idx.y;
  if (params.stage_b) { StageBScan(u, r); return; }
  const DeviceRow row = params.rows[r];
  const float tan_u = params.tangents[u];
  float3 d = make_float3(row.optical[0] + tan_u * row.line[0], row.optical[1] + tan_u * row.line[1],
                         row.optical[2] + tan_u * row.line[2]);
  const float inv = rsqrtf(d.x * d.x + d.y * d.y + d.z * d.z);
  d = make_float3(d.x * inv, d.y * inv, d.z * inv);
  unsigned lo = 0, hi = 0, hit = 0;
  optixTrace(params.handle, make_float3(0.f, row.origin_y, row.origin_z), d, 0.f, 1e3f, 0.f, 255,
             OPTIX_RAY_FLAG_DISABLE_ANYHIT, 0, 1, 0, lo, hi, hit);
  unsigned char code = 0;
  bool valid = false;
  double x = 0, q = 0;
  if (hit) {
    const double t = Join(lo, hi);
    x = row.origin_x + t * d.x;
    const double y = row.origin_y + t * d.y, z = row.origin_z + t * d.z;
    q = params.radius * atan2(y, z - params.axis_z);
    valid = x >= params.x_min && x <= params.x_max;
    if (valid) code = ssb::AlbedoCode(ssb::WallAlbedo(x, q));
  }
  if (!valid) atomicAdd(params.invalid + r, 1u);
  params.pixels[static_cast<size_t>(r) * params.width + u] = code;
  const int slot = params.debug_slot[u];
  if (slot >= 0) {
    double* out = params.debug_hits + (static_cast<size_t>(r) * params.debug_count + slot) * 2;
    out[0] = valid ? x : nan("");
    out[1] = valid ? q : nan("");
  }
}

extern "C" __global__ void __closesthit__wall() {
  const double t=optixGetRayTmax();
  optixSetPayload_0(Lo(t));optixSetPayload_1(Hi(t));optixSetPayload_2(1u);
  optixSetPayload_3(optixGetPrimitiveIndex());
}
namespace {
__device__ float3 Add(float3 a,float3 b) { return make_float3(a.x+b.x,a.y+b.y,a.z+b.z); }
__device__ float3 Sub(float3 a,float3 b) { return make_float3(a.x-b.x,a.y-b.y,a.z-b.z); }
__device__ float3 Mul(float3 a,float b) { return make_float3(a.x*b,a.y*b,a.z*b); }
__device__ float Dot(float3 a,float3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
__device__ float3 Cross(float3 a,float3 b) {return make_float3(a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x);}
__device__ float3 Unit(float3 a) { return Mul(a,rsqrtf(fmaxf(Dot(a,a),1e-20f))); }
__device__ double WrapQ(double q) {
  if(q>=params.tex_q0 && q<params.tex_q0+params.tex_period) return q;
  double f=fmod(q-params.tex_q0,params.tex_period);
  return params.tex_q0+(f<0?f+params.tex_period:f);
}
__device__ float4 Texel(const ssb::SurfaceTexel& t) {return make_float4(t.albedo/65535.f,t.roughness/255.f,t.nx/32767.f,t.nq/32767.f);}
__device__ float4 Mix(float4 a,float4 b,float w) {return make_float4(a.x+(b.x-a.x)*w,a.y+(b.y-a.y)*w,a.z+(b.z-a.z)*w,a.w+(b.w-a.w)*w);}
__device__ bool Surface(double x,double q,float4* result,unsigned* guard=nullptr) {
  q=WrapQ(q);
  double xp=(x-params.tex_x0)/params.tex_dx,qp=(q-params.tex_q0)/params.tex_dq;
  int ix=min(int(params.tiles_x)-1,max(0,int(floor(xp))/int(params.tile_core)));
  int iq=min(int(params.tiles_q)-1,max(0,int(floor(qp))/int(params.tile_core)));
  const auto* tile=params.tiles[iq*params.tiles_x+ix];if(!tile) return false;
  double u=xp-.5-ix*params.tile_core+params.tile_gutter,v=qp-.5-iq*params.tile_core+params.tile_gutter;
  int left=int(floor(u)),top=int(floor(v));
  if(left<0||top<0||left+1>=int(params.tile_side)||top+1>=int(params.tile_side)) return false;
  float fx=u-left,fy=v-top;
  auto a=Mix(Texel(tile[top*params.tile_side+left]),Texel(tile[top*params.tile_side+left+1]),fx);
  auto b=Mix(Texel(tile[(top+1)*params.tile_side+left]),Texel(tile[(top+1)*params.tile_side+left+1]),fx);
  *result=Mix(a,b,fy);
  if(guard) *guard=tile[top*params.tile_side+left].reserved | tile[top*params.tile_side+left+1].reserved |
                   tile[(top+1)*params.tile_side+left].reserved | tile[(top+1)*params.tile_side+left+1].reserved;
  return true;
}
// Edge band half width beside a crack of local radius r: fades out as the crack tapers.
__device__ float EdgeBand(float r) {return params.crack_edge_band*fminf(1.f,r/1e-4f);}
// Point classification: 2 inside the crack opening, 1 in its edge band, 0 outside.
__device__ int CrackPoint(double x,double q) {
  q=WrapQ(q);
  int ix=int(floor((x-params.crack_x0)/params.crack_cell)),iq=int(floor((q-params.crack_q0)/params.crack_cell));
  if(ix<0||iq<0||ix>=int(params.crack_nx)||iq>=int(params.crack_nq)) return 0;
  int best=0;
  for(int jq=max(0,iq-1);jq<=min(int(params.crack_nq)-1,iq+1);++jq)for(int jx=max(0,ix-1);jx<=min(int(params.crack_nx)-1,ix+1);++jx) {
    unsigned cell=jq*params.crack_nx+jx;
    for(unsigned k=params.crack_offsets[cell];k<params.crack_offsets[cell+1];++k) {
      auto s=params.cracks[params.crack_indices[k]];
      float dx=s.x1-s.x0,dq=s.q1-s.q0,px=float(x-s.x0),pq=float(q-s.q0);
      float t=fminf(1.f,fmaxf(0.f,(px*dx+pq*dq)/(dx*dx+dq*dq)));
      float radius=s.r0+(s.r1-s.r0)*t,d2=(px-t*dx)*(px-t*dx)+(pq-t*dq)*(pq-t*dq);
      if(d2<=radius*radius) return 2;
      float e=radius+EdgeBand(radius);if(d2<=e*e) best=1;
    }
  }
  return best;
}
__device__ float TriangleMargin(float3 point,unsigned primitive) {
  uint3 tri=params.triangles[primitive];float3 a=params.vertices[tri.x],b=params.vertices[tri.y],c=params.vertices[tri.z];
  float3 n=Unit(Cross(Sub(b,a),Sub(c,a)));
  float m0=fabsf(Dot(Cross(Sub(b,a),Sub(point,a)),n))/sqrtf(Dot(Sub(b,a),Sub(b,a)));
  float m1=fabsf(Dot(Cross(Sub(c,b),Sub(point,b)),n))/sqrtf(Dot(Sub(c,b),Sub(c,b)));
  float m2=fabsf(Dot(Cross(Sub(a,c),Sub(point,c)),n))/sqrtf(Dot(Sub(a,c),Sub(a,c)));
  return fminf(m0,fminf(m1,m2));
}
__device__ float CriticalMargin(float3 point,unsigned primitive) {
  unsigned mask=params.critical_edges[primitive];if(!mask)return 1e10f;
  uint3 tri=params.triangles[primitive];float3 v[3]={params.vertices[tri.x],params.vertices[tri.y],params.vertices[tri.z]};
  float3 n=Unit(Cross(Sub(v[1],v[0]),Sub(v[2],v[0])));float margin=1e10f;
  for(unsigned e=0;e<3;++e)if(mask&(1u<<e)) {
    float3 edge=Sub(v[(e+1)%3],v[e]);
    margin=fminf(margin,fabsf(Dot(Cross(edge,Sub(point,v[e])),n))/sqrtf(Dot(edge,edge)));
  }
  return margin;
}
__device__ bool Hit(const DeviceRow& row,float tangent,float scan_tangent,double* x,double* q,
                     float3* point,float3* direction,unsigned* primitive) {
  *direction=Unit(make_float3(row.optical[0]+tangent*row.line[0]+scan_tangent*row.scan[0],
                         row.optical[1]+tangent*row.line[1]+scan_tangent*row.scan[1],
                         row.optical[2]+tangent*row.line[2]+scan_tangent*row.scan[2]));
  unsigned lo=0,hi=0,hit=0,tri=0;
  float3 origin=make_float3(float(row.origin_x),row.origin_y,row.origin_z);
  optixTrace(params.handle,origin,*direction,0.f,1e3f,0.f,255,OPTIX_RAY_FLAG_DISABLE_ANYHIT,0,1,0,lo,hi,hit,tri);
  if(!hit) return false;
  double t=Join(lo,hi);
  // Use the actual float ray origin in a triangle scene; mixing in its unrounded
  // double origin would falsely claim hit accuracy absent from the traced ray.
  *x=double(origin.x)+t*direction->x;
  double y=origin.y+t*direction->y,z=origin.z+t*direction->z;
  *q=params.radius*atan2(y,z-params.axis_z);
  *point=make_float3(float(*x),float(y),float(z));*primitive=tri;
  return *x>=params.x_min&&*x<=params.x_max;
}
// Same finite-source sample positions for shading and the slot visibility guard.
// New COB mode uses 2x2 Gauss quadrature over an equivalent source patch; the
// custom lens angular redistribution remains the empirical beam envelope below.
__device__ float3 LampSample(const DeviceRow& row,unsigned i) {
  const float3 scan=make_float3(row.scan[0],row.scan[1],row.scan[2]);
  const float3 optical=make_float3(row.optical[0],row.optical[1],row.optical[2]);
  const float3 axis=Unit(Cross(scan,optical));
  float s=((i+.5f)/params.light_samples-.5f)*params.lamp_length,w=0;
  if(params.lamp_width>0) {
    s=(i&1?1.f:-1.f)*.288675134595f*params.lamp_length;
    w=(i&2?1.f:-1.f)*.288675134595f*params.lamp_width;
  }
  float3 centre=make_float3(float(row.origin_x)+params.lamp_axial,row.origin_y,row.origin_z);
  return Add(centre,Add(Mul(optical,params.lamp_radial),
                       Add(Mul(scan,params.lamp_tangential+w),Mul(axis,s))));
}
// Mean-normalised mortar detail tiled by wall (x, q), bilinear with wrap.
__device__ float Detail(double x,double q) {
  if(!params.filler) return 1.f;
  const double w=params.filler_width,h=params.filler_height;
  double u=fmod(x/params.filler_pitch-.5,w),v=fmod(q/params.filler_pitch-.5,h);if(u<0)u+=w;if(v<0)v+=h;
  int x0=int(u),y0=int(v);float fx=float(u-x0),fy=float(v-y0);
  auto at=[&](int i,int j){return float(params.filler[size_t((y0+j)%params.filler_height)*params.filler_width+(x0+i)%params.filler_width]);};
  float s=(at(0,0)*(1-fx)+at(1,0)*fx)*(1-fy)+(at(0,1)*(1-fx)+at(1,1)*fx)*fy;
  return float(s/params.filler_scale);
}
__device__ float Filler(double x,double q) {return params.filler_mean*Detail(x,q);}
// Groove walls/floor: darker concrete with low-contrast detail at an offset (not continuous
// with the lining texture seen through the joint opening).
__device__ float GrooveConcrete(double x,double q) {
  return params.groove_albedo*(1.f+params.groove_detail_contrast*(Detail(x+.371,q+.529)-1.f));
}
__device__ float Shade(const DeviceRow& row,float3 point,float3 view,unsigned primitive,double x,double q,float4 tex,float2 crack=make_float2(-1.f,0.f),unsigned visibility=~0u) {
  uint3 tri=params.triangles[primitive];
  float3 normal=Unit(Cross(Sub(params.vertices[tri.y],params.vertices[tri.x]),Sub(params.vertices[tri.z],params.vertices[tri.x])));
  if(Dot(normal,view)>0) normal=Mul(normal,-1.f); // double-sided optical lining, inward visible normal
  // Materials: 0 lining panel and chamfers (texture, normal map, cracks); 1 groove walls/floor
  // (dusty concrete, face normal); 2 joint filler (mortar); 3 gasket/void behind the contact gap.
  const unsigned material=params.face_material[primitive];
  bool joint=material!=0;
  float albedo=material==0 ? tex.x : material==1 ? GrooveConcrete(x,q) : material==2 ? Filler(x,q) : params.gap_albedo;
  if(!joint) {
    // Crack opening: dark but not black (interior ratio of the wall albedo with debris-like
    // variation); edge band slightly darker. Coverage from the footprint integral, or a point test.
    float core=crack.x,edge=crack.y;
    if(core<0) {int c=CrackPoint(x,q);core=c==2;edge=c==1;}
    if(core>0||edge>0) {
      float interior=params.crack_interior<0 ? .035f :   // legacy scenes: flat dark opening
        params.crack_interior*tex.x*(1.f+params.crack_interior_variation*(Detail(x*1.7+.131,q*1.7+.293)-1.f));
      albedo=tex.x*(1.f-params.crack_edge_darkening*edge)-core*(tex.x-interior);
    }
  }
  if(!params.light_enabled) return albedo;
  if(!joint) {
    float3 tangent=Unit(Sub(make_float3(1,0,0),Mul(normal,normal.x)));
    float3 bitangent=Unit(Cross(normal,tangent));
    // At theta=0 the positive q tangent is +y; inward normal cross +x is -y.
    bitangent=Mul(bitangent,-1.f);
    float nz=sqrtf(fmaxf(0.f,1.f-tex.z*tex.z-tex.w*tex.w));
    normal=Unit(Add(Mul(normal,nz),Add(Mul(tangent,tex.z),Mul(bitangent,tex.w))));
  }
  double dqx=q-row.optical_q;
  if(dqx>params.tex_period*.5) dqx-=params.tex_period;
  if(dqx<-params.tex_period*.5) dqx+=params.tex_period;
  float ax=2.f*float(x-row.origin_x-params.lamp_axial)/params.footprint_x,aq=2.f*float(dqx)/params.footprint_q;
  ax*=ax;ax*=ax;ax*=ax;aq*=aq;aq*=aq;aq*=aq;
  float beam=expf(-.69314718056f*(ax+aq));
  float intensity=0;
  const bool trace_shadows=visibility==~0u && params.shadows &&
    !(params.convex_panel_visibility && !joint && TriangleMargin(point,primitive)>.001f);
  for(unsigned i=0;i<params.light_samples;++i) {
    if(visibility!=~0u && !(visibility&(1u<<i)))continue;
    float3 light=LampSample(row,i);
    float3 delta=Sub(light,point);float distance=sqrtf(Dot(delta,delta));float3 L=Mul(delta,1.f/distance);
    // An inward panel far from its rim sees an axial emitter through the convex
    // tunnel interior. Slot floors/edges still require actual shadow rays. Enable
    // this only for the generated wall+outward slots, with no interior occluders.
    if(trace_shadows) {
      unsigned lo=0,hi=0,hit=0,ignored=0;
      // Offset towards the emitter, avoiding self hits without widening the seam.
      optixTrace(params.handle,Add(point,Mul(L,2e-5f)),L,0.f,fmaxf(0.f,distance-4e-5f),0.f,255,
                 OPTIX_RAY_FLAG_DISABLE_ANYHIT|OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT,0,1,0,lo,hi,hit,ignored);
      if(hit) continue;
    }
    float ndl=fmaxf(0.f,Dot(normal,L));
    // Relative rough diffuse approximation. Absolute photometry and
    // measured camera gain remain uncalibrated; no claim of lux or SNR.
    float rough=material==0?tex.y:material==2?params.filler_roughness:.9f;
    float diffuse=ndl*(1.f-.12f*rough*rough);
    intensity+=diffuse*float(params.radius*params.radius)/(distance*distance);
  }
  return albedo*beam*intensity/params.light_samples;
}
}
// Local affine footprint in metric (x,q); curvature across a ~0.2 mm pixel
// is negligible compared with the float triangle intersection precision.
__device__ double2 MetricDelta(float3 delta,float3 point) {
  double y=point.y,z=double(point.z)-params.axis_z;
  return make_double2(delta.x,params.radius*(z*delta.y-y*delta.z)/(y*y+z*z));
}
__device__ float3 PixelWorldDelta(const DeviceRow& row,float3 point,float tangent,unsigned primitive,bool across) {
  uint3 tri=params.triangles[primitive];
  float3 normal=Unit(Cross(Sub(params.vertices[tri.y],params.vertices[tri.x]),Sub(params.vertices[tri.z],params.vertices[tri.x])));
  float3 raw=make_float3(row.optical[0]+tangent*row.line[0],row.optical[1]+tangent*row.line[1],row.optical[2]+tangent*row.line[2]);
  float3 axis=across?make_float3(row.line[0],row.line[1],row.line[2]):make_float3(row.scan[0],row.scan[1],row.scan[2]);
  float3 origin=make_float3(float(row.origin_x),row.origin_y,row.origin_z);
  float travel=Dot(Sub(point,origin),normal)/Dot(raw,normal);
  return Mul(Sub(axis,Mul(raw,Dot(axis,normal)/Dot(raw,normal))),travel*params.pixel_step);
}
// On a single slot face, test the corners of the complete space/time footprint.
// This guard is specific to the generated convex lining and wide rectangular
// slots, without interior occluders. Any visibility change retains full rays.
__device__ bool SlotVisibility(const DeviceRow& first,const DeviceRow& last,float3 point,
                               float3 a,float3 b,float3 motion,unsigned* visibility) {
  unsigned mask=0;
  for(unsigned i=0;i<params.light_samples;++i) {
    int previous=-1;
    for(unsigned t=0;t<2;++t)for(unsigned corner=0;corner<4;++corner) {
      float3 p=Add(point,Add(Mul(a,corner&1?.5f:-.5f),Add(Mul(b,corner&2?.5f:-.5f),Mul(motion,t?.5f:-.5f))));
      // Extrapolate sample positions to the actual exposure endpoints.
      float3 p0=LampSample(first,i),p1=LampSample(last,i);
      float3 light=t?Add(p1,Mul(Sub(p1,p0),.25f)):Add(p0,Mul(Sub(p0,p1),.25f));
      float3 delta=Sub(light,p);float distance=sqrtf(Dot(delta,delta));float3 L=Mul(delta,1.f/distance);
      unsigned lo=0,hi=0,hit=0,ignored=0;
      optixTrace(params.handle,Add(p,Mul(L,2e-5f)),L,0.f,fmaxf(0.f,distance-4e-5f),0.f,255,
                 OPTIX_RAY_FLAG_DISABLE_ANYHIT|OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT,0,1,0,lo,hi,hit,ignored);
      if(previous>=0 && int(hit)!=previous)return false;
      previous=int(hit);
    }
    if(previous==0)mask|=1u<<i;
  }
  *visibility=mask;return true;
}
// Crack coverage over the pixel's space/time footprint. Every nearby segment is treated as a
// band (exact box/linear-motion integral of its two half-planes, tapered radius) over the part
// of the footprint within its extent; the union is approximated by the maximum. Consecutive
// segments of one polyline need no special case, so bends do not fall back to sampling.
// Returns (opening coverage, edge-band coverage).
__device__ float2 IntegratedCrack(double x,double q,double2 a,double2 b,double2 motion) {
  q=WrapQ(q);
  const float band=params.crack_edge_band;
  double hx=.5*(fabs(a.x)+fabs(b.x)+fabs(motion.x))+band;
  double hq=.5*(fabs(a.y)+fabs(b.y)+fabs(motion.y))+band;
  int lo_x=max(0,int(floor((x-hx-params.crack_x0)/params.crack_cell)));
  int hi_x=min(int(params.crack_nx)-1,int(floor((x+hx-params.crack_x0)/params.crack_cell)));
  int lo_q=max(0,int(floor((q-hq-params.crack_q0)/params.crack_cell)));
  int hi_q=min(int(params.crack_nq)-1,int(floor((q+hq-params.crack_q0)/params.crack_cell)));
  float core=0,outer=0;
  const float ax=a.x,ay=a.y,bx=b.x,by=b.y,mx=motion.x,my=motion.y;
  for(int iq=lo_q;iq<=hi_q;++iq)for(int ix=lo_x;ix<=hi_x;++ix) {
    unsigned cell=iq*params.crack_nx+ix;
    for(unsigned k=params.crack_offsets[cell];k<params.crack_offsets[cell+1];++k) {
      auto s=params.cracks[params.crack_indices[k]];
      float dx=s.x1-s.x0,dq=s.q1-s.q0,length=sqrtf(dx*dx+dq*dq);if(length<=0)continue;
      float lx=dx/length,lq=dq/length,px=float(x-s.x0),pq=float(q-s.q0);
      float along=px*lx+pq*lq,perp=-px*lq+pq*lx;
      float pa=-ax*lq+ay*lx,pb=-bx*lq+by*lx,pt=-mx*lq+my*lx;
      float aa=ax*lx+ay*lq,ab=bx*lx+by*lq,at=mx*lx+my*lq;
      float along_half=.5f*(fabsf(aa)+fabsf(ab)+fabsf(at)),perp_half=.5f*(fabsf(pa)+fabsf(pb)+fabsf(pt));
      float rmax=fmaxf(s.r0,s.r1);
      if(fabsf(perp)>rmax+EdgeBand(rmax)+perp_half || along+along_half<0 || along-along_half>length)continue;
      float slope=(s.r1-s.r0)/length,r=s.r0+slope*fminf(length,fmaxf(0.f,along));
      float c=ssb::BoxHalfPlaneT<float>(pa-slope*aa,pb-slope*ab,pt-slope*at,perp-r)-
              ssb::BoxHalfPlaneT<float>(pa+slope*aa,pb+slope*ab,pt+slope*at,perp+r);
      float e=r+EdgeBand(r);
      float o=ssb::BoxHalfPlaneT<float>(pa,pb,pt,perp-e)-ssb::BoxHalfPlaneT<float>(pa,pb,pt,perp+e);
      core=fmaxf(core,c);outer=fmaxf(outer,o);
    }
  }
  core=fminf(1.f,fmaxf(0.f,core));
  return make_float2(core,fmaxf(0.f,fminf(1.f,outer)-core));
}
__device__ bool IntegratedPixel(unsigned u,unsigned r,const DeviceRow& centre,double x,double q,
                                float3 point,float3 direction,unsigned primitive) {
  // Return false at real material/geometry boundaries: retain the full-ray path.
  // Coplanar diagonals and smooth cylindrical facets do not force oversampling.
  if(params.light_enabled && params.shadows && !params.convex_panel_visibility)return false;
  float3 world_a=PixelWorldDelta(centre,point,params.tangents[u],primitive,true);
  float3 world_b=PixelWorldDelta(centre,point,params.tangents[u],primitive,false);
  double2 a=MetricDelta(world_a,point),b=MetricDelta(world_b,point);
  if(CriticalMargin(point,primitive)<.0006f)return false;
  float3 points[3],dirs[3];double xs[3],qs[3];unsigned ids[3];
  for(unsigned t=0;t<3;++t) {
    if(t==1){points[t]=point;dirs[t]=direction;xs[t]=x;qs[t]=q;ids[t]=primitive;}
    else if(!Hit(params.rows[r*params.row_stride+1+t],params.tangents[u],0,&xs[t],&qs[t],&points[t],&dirs[t],&ids[t]))return false;
    if(params.face_material[ids[t]]!=params.face_material[primitive] || CriticalMargin(points[t],ids[t])<.0006f)return false;
  }
  double dq=qs[2]-qs[0];if(dq>params.tex_period*.5)dq-=params.tex_period;if(dq<-params.tex_period*.5)dq+=params.tex_period;
  double2 motion=make_double2((xs[2]-xs[0])*1.5,dq*1.5);
  unsigned visibility=~0u;
  if(params.light_enabled && params.shadows) {
    visibility=(1u<<params.light_samples)-1;
    if(params.face_material[primitive] &&
       !SlotVisibility(params.rows[r*params.row_stride+1],params.rows[r*params.row_stride+3],point,
                       world_a,world_b,Mul(Sub(points[2],points[0]),1.5f),&visibility))return false;
  }
  float2 coverage=params.face_material[primitive]?make_float2(0.f,0.f):IntegratedCrack(x,q,a,b,motion);
  float sum=0;
  const unsigned n=params.texture_footprint_samples;
  #pragma unroll
  for(unsigned t=0;t<3;++t) {
    const DeviceRow& row=params.rows[r*params.row_stride+1+t];
    if(n==1) {
      float4 tex;if(!Surface(xs[t],qs[t],&tex))return false;
      sum+=Shade(row,points[t],dirs[t],ids[t],xs[t],qs[t],tex,coverage,visibility);
      continue;
    }
    // Box-integrate the texture over the pixel footprint without extra rays:
    // stratified taps on the local affine (x,q) footprint around this frame's hit.
    float4 mean=make_float4(0,0,0,0);float shaded=0;
    for(unsigned sy=0;sy<n;++sy)for(unsigned sx=0;sx<n;++sx) {
      double fx=(sx+.5)/n-.5,fy=(sy+.5)/n-.5,tx=xs[t]+a.x*fx+b.x*fy,tq=qs[t]+a.y*fx+b.y*fy;
      float4 tex;if(!Surface(tx,tq,&tex))return false;
      if(params.texture_prefilter)mean=make_float4(mean.x+tex.x,mean.y+tex.y,mean.z+tex.z,mean.w+tex.w);
      else shaded+=Shade(row,points[t],dirs[t],ids[t],tx,tq,tex,coverage,visibility);
    }
    const float w=1.f/(n*n);
    sum+=params.texture_prefilter?Shade(row,points[t],dirs[t],ids[t],xs[t],qs[t],make_float4(mean.x*w,mean.y*w,mean.z*w,mean.w*w),coverage,visibility):shaded*w;
  }
  float value=sum/3*params.response_gain;
  params.pixels[size_t(r)*params.width+u]=static_cast<unsigned char>(fminf(255.f,fmaxf(0.f,value*255.f+.5f)));
  return true;
}
__device__ void StageBScan(unsigned u,unsigned r) {
  const DeviceRow centre=params.rows[r*params.row_stride];
  double x=0,q=0;float3 point,direction;unsigned primitive=0;
  bool valid=Hit(centre,params.tangents[u],0,&x,&q,&point,&direction,&primitive);
  int debug=params.debug_slot[u];
  if(debug>=0) {
    double* out=params.debug_hits+(size_t(r)*params.debug_count+debug)*2;
    out[0]=valid?x:nan("");out[1]=valid?q:nan("");
  }
  if(valid && params.integrated_cracks && IntegratedPixel(u,r,centre,x,q,point,direction,primitive))return;
  unsigned area=params.area_samples;
  double footprint_margin=.0005+2*params.pixel_step*params.radius;
  for(unsigned time=0;time<params.time_samples;++time) {
    auto frame=params.rows[r*params.row_stride+1+time];
    footprint_margin=fmax(footprint_margin,.0005+2*params.pixel_step*params.radius+
      fabs(frame.origin_x-centre.origin_x)+params.radius*sqrt(
      double(frame.optical[0]-centre.optical[0])*(frame.optical[0]-centre.optical[0])+
      double(frame.optical[1]-centre.optical[1])*(frame.optical[1]-centre.optical[1])+
      double(frame.optical[2]-centre.optical[2])*(frame.optical[2]-centre.optical[2])));
  }
  // Only reduce area sampling on smooth background, away from authored crack
  // vectors and triangle rims. Time integration remains active for every pixel.
  if(valid && params.adaptive_area && params.face_material[primitive]==0 && TriangleMargin(point,primitive)>.0005f &&
     footprint_margin<.002) {
    float4 tex;unsigned guard=1;
    if(Surface(x,q,&tex,&guard) && !guard) area=1;
  }
  float sum=0;
  for(unsigned t=0;t<params.time_samples;++t) {
    DeviceRow row=params.rows[r*params.row_stride+1+t];
    for(unsigned sy=0;sy<area;++sy) for(unsigned sx=0;sx<area;++sx) {
      float tangent=params.tangents[u]+((sx+.5f)/area-.5f)*params.pixel_step;
      float scan=((sy+.5f)/area-.5f)*params.pixel_step;
      double hx,hq;float3 hp,hd;unsigned tri;float4 tex;
      bool ok=Hit(row,tangent,scan,&hx,&hq,&hp,&hd,&tri);
      unsigned reason=ok?0:1;
      if(ok && !Surface(hx,hq,&tex)) {ok=false;reason=2;}
      if(!ok) {
        valid=false;
        atomicOr(params.invalid_flags+r,reason);
        atomicMin(params.invalid_column+r,u);
        continue;
      }
      sum+=Shade(row,hp,hd,tri,hx,hq,tex);
    }
  }
  if(!valid) atomicAdd(params.invalid+r,1u);
  float value=sum/(params.time_samples*area*area)*params.response_gain;
  params.pixels[size_t(r)*params.width+u]=static_cast<unsigned char>(fminf(255.f,fmaxf(0.f,value*255.f+.5f)));
}
