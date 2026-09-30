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
  T lengths[3]={fabs(a),fabs(b),fabs(c)},scale=fmax(lengths[0],fmax(lengths[1],lengths[2]));
  if(scale==0) return bias<=0?T(1):T(0);
  unsigned n=0;T total=0,product=1;
  for(unsigned i=0;i<3;++i) if(lengths[i]>scale*T(1e-5)) {
    lengths[n++]=lengths[i];total+=lengths[i];product*=lengths[i];
  }
  T z=total*T(.5)-bias;
  if(z<=0) return 0;
  if(z>=total) return 1;
  bool reflect=z>total*T(.5);if(reflect) z=total-z;
  T sum=0;
  for(unsigned bits=0;bits<(1u<<n);++bits) {
    T v=z;unsigned parity=0;
    for(unsigned i=0;i<n;++i) if(bits&(1u<<i)) {v-=lengths[i];++parity;}
    if(v>0) {T power=v;for(unsigned i=1;i<n;++i) power*=v;sum+=(parity&1)?-power:power;}
  }
  T result=sum/(product*(n==3?T(6):n==2?T(2):T(1)));
  result=fmin(T(1),fmax(T(0),result));return reflect?1-result:result;
}
SSB_INTEGRAL_HD inline double BoxHalfPlane(double a,double b,double c,double bias) {return BoxHalfPlaneT<double>(a,b,c,bias);}
}
#undef SSB_INTEGRAL_HD
