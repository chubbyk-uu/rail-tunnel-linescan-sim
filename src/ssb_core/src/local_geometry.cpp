#include "ssb_core/local_geometry.hpp"
#include <cmath>
#include <algorithm>
#include <map>
#include <stdexcept>

namespace ssb {
namespace {
using Point=std::array<double,3>;
Point Sub(Point a,Point b) { return {a[0]-b[0],a[1]-b[1],a[2]-b[2]}; }
Point Cross(Point a,Point b) { return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]}; }
double Dot(Point a,Point b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
std::vector<Point> Clip(const std::vector<Point>& polygon,double plane,bool lower) {
  std::vector<Point> out;
  for(size_t i=0;i<polygon.size();++i) {
    const auto a=polygon[i],b=polygon[(i+1)%polygon.size()];
    const bool ai=lower?a[0]>=plane:a[0]<=plane,bi=lower?b[0]>=plane:b[0]<=plane;
    if(ai) out.push_back(a);
    if(ai!=bi) {
      const double t=(plane-a[0])/(b[0]-a[0]);
      out.push_back({plane,a[1]+t*(b[1]-a[1]),a[2]+t*(b[2]-a[2])});
    }
  }
  return out;
}
bool OnEdge(Point p,Point a,Point b) {
  const auto edge=Sub(b,a),delta=Sub(p,a),cross=Cross(edge,delta);
  const double length=Dot(edge,edge),t=Dot(delta,edge)/length;
  return Dot(cross,cross)<=1e-18*length && t>=-1e-9 && t<=1+1e-9;
}
struct Piece { std::array<Point,3> points; unsigned material,critical; };
}

LocalGeometry LocalizeGeometry(const StageBAssets& a) {
  std::map<long long,std::vector<Piece>> groups;
  for(size_t i=0;i<a.triangles.size();++i) {
    const auto tri=a.triangles[i];
    std::array<Point,3> p;
    const unsigned ids[3]={tri.a,tri.b,tri.c};
    for(int j=0;j<3;++j) {auto v=a.vertices[ids[j]];p[j]={v.x,v.y,v.z};}
    const double lo=std::min({p[0][0],p[1][0],p[2][0]}),hi=std::max({p[0][0],p[1][0],p[2][0]});
    const auto first=static_cast<long long>(std::floor((lo-kOpticalChunkOverlap)/kOpticalChunkLength));
    const auto last=static_cast<long long>(std::floor((hi+kOpticalChunkOverlap)/kOpticalChunkLength));
    if(last-first>1000) throw std::runtime_error("optical triangle exceeds supported chunk span");
    for(auto chunk=first;chunk<=last;++chunk) {
      auto polygon=Clip(Clip({p.begin(),p.end()},chunk*kOpticalChunkLength-kOpticalChunkOverlap,true),
                        (chunk+1)*kOpticalChunkLength+kOpticalChunkOverlap,false);
      if(polygon.size()<3) continue;
      for(size_t j=1;j+1<polygon.size();++j) {
        Piece piece{{polygon[0],polygon[j],polygon[j+1]},a.face_material[i],0};
        const auto n=Cross(Sub(piece.points[1],piece.points[0]),Sub(piece.points[2],piece.points[0]));
        if(Dot(n,n)<1e-24) continue;
        const unsigned mask=a.critical_edges.empty()?0:a.critical_edges[i];
        for(unsigned e=0;e<3;++e) for(unsigned old=0;old<3;++old)
          if((mask&(1u<<old)) && OnEdge(piece.points[e],p[old],p[(old+1)%3]) &&
             OnEdge(piece.points[(e+1)%3],p[old],p[(old+1)%3])) piece.critical|=1u<<e;
        groups[chunk].push_back(piece);
      }
    }
  }
  LocalGeometry result;
  for(const auto& [index,pieces]:groups) {
    const double origin=(index+.5)*kOpticalChunkLength;
    result.chunks.push_back({origin,unsigned(result.triangles.size()),unsigned(pieces.size())});
    for(const auto& piece:pieces) {
      const unsigned base=result.vertices.size();
      for(auto p:piece.points) result.vertices.push_back({float(p[0]-origin),float(p[1]),float(p[2])});
      result.triangles.push_back({base,base+1,base+2});result.material.push_back(piece.material);
      result.critical_edges.push_back(piece.critical);result.primitive_origin_x.push_back(origin);
    }
  }
  if(result.triangles.size()>2400000 || result.chunks.size()>1000)
    throw std::runtime_error("localized optical geometry exceeds budget");
  return result;
}
}  // namespace ssb
