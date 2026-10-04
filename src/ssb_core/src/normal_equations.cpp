#include <algorithm>
#include <cmath>
#include <cstdint>

namespace {
template<class Index>
int Accumulate(int columns, int64_t first, int64_t last, int64_t nnz,
               const Index* offsets, const Index* indices, const double* values,
               const double* residual, double* normal, double* gradient) {
  std::fill(normal, normal+int64_t(columns)*columns, 0.);
  std::fill(gradient, gradient+columns, 0.);
  for (int64_t row=first; row<last; ++row) {
    const int64_t begin=offsets[row], end=offsets[row+1];
    if (begin<0 || end<begin || end>nnz || !std::isfinite(residual[row])) return 2;
    int64_t previous=-1;
    for (int64_t i=begin; i<end; ++i) {
      const int64_t a=indices[i]; const double value=values[i];
      if (a<=previous || a>=columns || !std::isfinite(value)) return 2;
      previous=a;
      gradient[a]+=value*residual[row];
      for (int64_t j=begin; j<=i; ++j)
        normal[a*columns+indices[j]]+=value*values[j];
    }
  }
  for (int a=0; a<columns; ++a) {
    if (!std::isfinite(gradient[a])) return 2;
    for (int b=0; b<=a; ++b) {
      const double value=normal[int64_t(a)*columns+b];
      if (!std::isfinite(value)) return 2;
      normal[int64_t(b)*columns+a]=value;
    }
  }
  return 0;
}
}

extern "C" int ssb_normal_abi() { return 1; }
extern "C" int ssb_normal_accumulate(int columns, int64_t first, int64_t last, int64_t rows,
    int64_t nnz, int bits, const void* offsets, const void* indices, const double* values,
    const double* residual, double* normal, double* gradient) {
  if (columns<1 || columns>2048 || first<0 || last<first || last>rows || nnz<0 ||
      !offsets || !indices || !values || !residual || !normal || !gradient) return 1;
  if (bits==32)
    return Accumulate(columns,first,last,nnz,static_cast<const int32_t*>(offsets),
                      static_cast<const int32_t*>(indices),values,residual,normal,gradient);
  if (bits==64)
    return Accumulate(columns,first,last,nnz,static_cast<const int64_t*>(offsets),
                      static_cast<const int64_t*>(indices),values,residual,normal,gradient);
  return 1;
}
