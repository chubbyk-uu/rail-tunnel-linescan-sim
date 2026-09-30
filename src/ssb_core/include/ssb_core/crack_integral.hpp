#pragma once
#include <cmath>

#ifdef __CUDACC__
#define SSB_INTEGRAL_HD __host__ __device__
#else
#define SSB_INTEGRAL_HD
#endif
namespace ssb {
// Volume fraction of a*s+b*t+c*u+bias <= 0, with independent uniform
// coordinates in [-.5,.5]. Exact box/linear-motion integration of a half-plane.
template<class T> SSB_INTEGRAL_HD inline T BoxHalfPlaneT(T a,T b,T c,T bias) {
  T lengths[3]={a<0?-a:a,b<0?-b:b,c<0?-c:c};
  // Sort and normalise first. Direct inclusion/exclusion of cubic powers loses
  // most float precision when a projected footprint dimension is almost zero.
  if(lengths[0]<lengths[1]) {T t=lengths[0];lengths[0]=lengths[1];lengths[1]=t;}
  if(lengths[1]<lengths[2]) {T t=lengths[1];lengths[1]=lengths[2];lengths[2]=t;}
  if(lengths[0]<lengths[1]) {T t=lengths[0];lengths[0]=lengths[1];lengths[1]=t;}
  T scale=lengths[0];
  if(scale==0) return bias<=0?T(1):T(0);
  unsigned n=0;T total=0;
  for(unsigned i=0;i<3;++i) if(lengths[i]>scale*T(1e-5)) {
    lengths[n]=lengths[i]/scale;total+=lengths[n++];
  }
  T z=total*T(.5)-bias/scale;
  if(z<=0) return 0;
  if(z>=total) return 1;
  bool reflect=z>total*T(.5);if(reflect) z=total-z;
  T result;
  if(n==1) result=z;
  else if(n==2) {
    T m=lengths[1];
    result=z>=m?z-m*T(.5):z*z/(T(2)*m);
  } else {
    T m=lengths[1],s=lengths[2];
    if(z>=m+s && z<=1) result=z-(m+s)*T(.5);
    else {
      // Difference of positive cubics factored by the smallest dimension.
      // In the reflected half, z < 1+m, so the fourth term is always zero.
      auto difference=[&](T v) {
        if(v<=0) return T(0);
        return v>s?s*(T(3)*v*(v-s)+s*s):v*v*v;
      };
      result=(difference(z)-difference(z-m)-difference(z-T(1)))/(T(6)*m*s);
    }
  }
  result=fmin(T(1),fmax(T(0),result));return reflect?1-result:result;
}
SSB_INTEGRAL_HD inline double BoxHalfPlane(double a,double b,double c,double bias) {return BoxHalfPlaneT<double>(a,b,c,bias);}
}
#undef SSB_INTEGRAL_HD
