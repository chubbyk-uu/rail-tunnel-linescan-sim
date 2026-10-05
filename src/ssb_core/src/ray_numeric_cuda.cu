#include "ssb_core/ray_numeric.hpp"

#include <cuda_runtime.h>
#include <array>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
thread_local std::string last_error;

void Check(cudaError_t status) {
  if (status!=cudaSuccess) throw std::runtime_error(cudaGetErrorString(status));
}

struct Sources {
  const double* axes[2];
  const double* phases[2];
  const double* tangents[2];
  const double* weights[2];
  const double* corrections[2];
};

struct Context {
  int64_t count, nnz;
  int fields, block_count;
  double radius, height;
  size_t bytes=0, budget;
  std::vector<void*> allocations;
  Sources sources{};
  int64_t* starts=nullptr;
  ssb::numeric::JacobianBlock* blocks=nullptr;
  int64_t* scale_positions=nullptr;
  double* scale_derivative=nullptr;
  double* current=nullptr;
  double* pitch=nullptr;
  double* jacobian=nullptr;
  double* difference=nullptr;
  int* columns=nullptr;
  double* coefficients=nullptr;
  int coefficient_count, scale_index;
  double coefficient_scale, scale_reference;
  bool parameters_valid=false;
  int* error=nullptr;

  ~Context() { for (void* pointer:allocations) cudaFree(pointer); }

  template<class T> T* Allocate(size_t count) {
    if (count>budget/sizeof(T) || count*sizeof(T)>budget-bytes)
      throw std::runtime_error("CUDA public ray allocation budget exceeded");
    void* pointer=nullptr;
    Check(cudaMalloc(&pointer,count*sizeof(T)));
    allocations.push_back(pointer); bytes+=count*sizeof(T);
    return static_cast<T*>(pointer);
  }

  template<class T> T* Upload(const T* source, size_t count) {
    T* target=Allocate<T>(count);
    Check(cudaMemcpy(target,source,count*sizeof(T),cudaMemcpyHostToDevice));
    return target;
  }

  void Parameters(const double* parameters);

  void Reset() {
    Check(cudaMemset(error,0,sizeof(int)));
  }

  void Finish() {
    Check(cudaGetLastError());
    int failure=0;
    Check(cudaMemcpy(&failure,error,sizeof(int),cudaMemcpyDeviceToHost));
    if (failure) throw std::runtime_error("invalid CUDA public ray/Jacobian geometry");
  }
};

__global__ void ParametersKernel(int64_t count, int fields, Sources sources,
    const ssb::numeric::JacobianBlock* blocks, int block_count, const int* columns,
    const double* coefficients, double scale, int scale_index, double reference, int* error) {
  const int64_t row=int64_t(blockIdx.x)*blockDim.x+threadIdx.x;
  if (row>=count) return;
  for (int block_index=0; block_index<block_count; ++block_index) {
    const auto& block=blocks[block_index];
    for (int group=0; group<block.groups; ++group) {
      double value=0.;
      for (int spline=0; spline<4; ++spline) {
        const int64_t index=(row*block.groups+group)*4+spline;
        value+=block.parameter_basis[index]*coefficients[columns[block.inverse[index]]];
      }
      value*=scale;
      if (!std::isfinite(value)) { atomicExch(error,1); continue; }
      for (int native=0; native<4; ++native) {
        if (!(block.masks[group] & (1U<<native))) continue;
        const int64_t index=4*row+native;
        const double relative=(block.field==0 && scale_index>=0) ?
            scale*coefficients[scale_index]*(sources.axes[block.side][index]-reference) : 0.;
        const_cast<double*>(sources.corrections[block.side])[fields*index+block.field]=value+relative;
      }
    }
  }
}

void Context::Parameters(const double* parameters) {
  Reset();
  if (!parameters) {
    if (!parameters_valid) throw std::runtime_error("CUDA corrections have not been initialized");
    return;
  }
  Check(cudaMemcpy(coefficients,parameters,coefficient_count*sizeof(double),cudaMemcpyHostToDevice));
  ParametersKernel<<<(count+127)/128,128>>>(count,fields,sources,blocks,block_count,columns,coefficients,
                                         coefficient_scale,scale_index,scale_reference,error);
  parameters_valid=true;
}

__global__ void DifferenceKernel(int64_t count, int fields, double radius, double height,
    Sources sources, double* output, int* error) {
  const int64_t row=int64_t(blockIdx.x)*blockDim.x+threadIdx.x;
  if (row<count && !ssb::numeric::DifferenceRow(row,fields,radius,height,sources.axes,sources.phases,
      sources.tangents,sources.weights,sources.corrections,output)) atomicExch(error,1);
}

__global__ void JacobianKernel(int64_t count, int fields, double radius, double height,
    Sources sources, const int64_t* starts, const ssb::numeric::JacobianBlock* blocks, int block_count,
    const int64_t* scale_positions, const double* scale_derivative,
    const double* current, const double* pitch, double* output, int* error) {
  const int64_t row=int64_t(blockIdx.x)*blockDim.x+threadIdx.x;
  if (row<count && !ssb::numeric::JacobianRow(row,fields,radius,height,sources.axes,sources.phases,
      sources.tangents,sources.weights,sources.corrections,starts,blocks,block_count,scale_positions,
      scale_derivative,current,pitch,output)) atomicExch(error,1);
}
}

extern "C" {
int ssb_ray_cuda_abi() { return 2; }
const char* ssb_ray_cuda_error() { return last_error.c_str(); }

void* ssb_ray_cuda_create(int64_t count, int fields, double radius, double height,
    const double* const* axes, const double* const* phases, const double* const* tangents,
    const double* const* weights, const int64_t* starts,
    const ssb::numeric::JacobianBlock* blocks, int block_count,
    const int64_t* scale_positions, const double* scale_derivative,
    const int* columns, int coefficient_count, double coefficient_scale,
    int scale_index, double scale_reference, size_t budget) {
  try {
    if (count<1 || count>2100000 || fields<4 || fields>6 || !std::isfinite(radius) || radius<=0. ||
        !std::isfinite(height) || height<=0. || !axes || !phases || !tangents || !weights || !starts ||
        !blocks || block_count!=2*fields || starts[0]!=0 || !columns || coefficient_count<1 ||
        coefficient_count>2048 || !(coefficient_scale>0.) || !std::isfinite(coefficient_scale) ||
        scale_index < -1 || scale_index>=coefficient_count || !std::isfinite(scale_reference) ||
        (scale_positions && !scale_derivative)) throw std::runtime_error("invalid CUDA ray plan");
    for (int64_t row=0; row<count; ++row)
      if (starts[row+1]<=starts[row] || starts[row+1]-starts[row]>193)
        throw std::runtime_error("invalid CUDA ray CSR pattern");
    auto context=std::make_unique<Context>();
    auto& c=*context;
    c.count=count; c.fields=fields; c.radius=radius; c.height=height;
    c.nnz=starts[count]; c.block_count=block_count; c.budget=budget;
    c.coefficient_count=coefficient_count; c.coefficient_scale=coefficient_scale;
    c.scale_index=scale_index; c.scale_reference=scale_reference;
    for (int64_t index=0; index<c.nnz; ++index)
      if (columns[index]<0 || columns[index]>=coefficient_count) throw std::runtime_error("invalid CUDA spline column");
    c.columns=c.Upload(columns,c.nnz); c.coefficients=c.Allocate<double>(coefficient_count);
    for (int side=0; side<2; ++side) {
      if (!axes[side] || !phases[side] || !tangents[side] || !weights[side])
        throw std::runtime_error("missing CUDA public rays");
      c.sources.axes[side]=c.Upload(axes[side],4*count);
      c.sources.phases[side]=c.Upload(phases[side],4*count);
      c.sources.tangents[side]=c.Upload(tangents[side],4*count);
      c.sources.weights[side]=c.Upload(weights[side],4*count);
      c.sources.corrections[side]=c.Allocate<double>(4*count*fields);
    }
    c.starts=c.Upload(starts,count+1);
    std::vector<ssb::numeric::JacobianBlock> device_blocks;
    bool seen[2][6]={{false}};
    for (int index=0; index<block_count; ++index) {
      auto block=blocks[index];
      if (block.side<0 || block.side>1 || block.field<0 || block.field>=fields ||
          block.groups<1 || block.groups>4 || !block.basis || !block.inverse || !block.parameter_basis ||
          (block.sign!=-1. && block.sign!=1.)) throw std::runtime_error("invalid CUDA derivative block");
      if (seen[block.side][block.field]) throw std::runtime_error("duplicate CUDA spline field");
      seen[block.side][block.field]=true;
      uint32_t coverage=0;
      for (int group=0; group<block.groups; ++group) {
        if (!block.masks[group] || block.masks[group]>15 || (coverage & block.masks[group]))
          throw std::runtime_error("invalid CUDA pixel group");
        coverage|=block.masks[group];
      }
      if (coverage!=15) throw std::runtime_error("incomplete CUDA pixel groups");
      const size_t entries=count*block.groups*4;
      for (int64_t row=0; row<count; ++row)
        for (int entry=0; entry<block.groups*4; ++entry) {
          const int64_t index=row*block.groups*4+entry, position=block.inverse[index];
          if (position<starts[row] || position>=starts[row+1] ||
              !std::isfinite(block.basis[index]) || !std::isfinite(block.parameter_basis[index]))
            throw std::runtime_error("invalid CUDA spline basis/index");
        }
      block.basis=c.Upload(block.basis,entries);
      block.inverse=c.Upload(block.inverse,entries);
      block.parameter_basis=c.Upload(block.parameter_basis,entries);
      device_blocks.push_back(block);
    }
    c.blocks=c.Upload(device_blocks.data(),device_blocks.size());
    if (scale_positions) {
      c.scale_positions=c.Upload(scale_positions,count);
      c.scale_derivative=c.Upload(scale_derivative,count);
    }
    c.current=c.Allocate<double>(2*count); c.pitch=c.Allocate<double>(2);
    c.jacobian=c.Allocate<double>(2*c.nnz); c.difference=c.Allocate<double>(2*count);
    c.error=c.Allocate<int>(1);
    return context.release();
  } catch (const std::exception& error) { last_error=error.what(); return nullptr; }
}

void ssb_ray_cuda_destroy(void* handle) { delete static_cast<Context*>(handle); }
size_t ssb_ray_cuda_bytes(void* handle) { return static_cast<Context*>(handle)->bytes; }

int ssb_ray_cuda_difference(void* handle, const double* coefficients, double* output) {
  try {
    if (!handle || !output) throw std::runtime_error("missing CUDA ray output");
    auto& c=*static_cast<Context*>(handle);
    c.Parameters(coefficients);
    DifferenceKernel<<<(c.count+127)/128,128>>>(c.count,c.fields,c.radius,c.height,c.sources,c.difference,c.error);
    c.Finish();
    Check(cudaMemcpy(output,c.difference,c.count*2*sizeof(double),cudaMemcpyDeviceToHost));
    return 0;
  } catch (const std::exception& error) { last_error=error.what(); return 1; }
}

int ssb_ray_cuda_jacobian(void* handle, const double* coefficients,
    const double* current, const double* pitch, double* output) {
  try {
    if (!handle || !current || !pitch || !output || !std::isfinite(pitch[0]) ||
        !std::isfinite(pitch[1]) || pitch[0]<=0. || pitch[1]<=0.) throw std::runtime_error("invalid CUDA ray weights");
    auto& c=*static_cast<Context*>(handle);
    c.Parameters(coefficients);
    Check(cudaMemcpy(c.current,current,c.count*2*sizeof(double),cudaMemcpyHostToDevice));
    Check(cudaMemcpy(c.pitch,pitch,2*sizeof(double),cudaMemcpyHostToDevice));
    JacobianKernel<<<(c.count+127)/128,128>>>(c.count,c.fields,c.radius,c.height,c.sources,c.starts,c.blocks,
        c.block_count,c.scale_positions,c.scale_derivative,c.current,c.pitch,c.jacobian,c.error);
    c.Finish();
    Check(cudaMemcpy(output,c.jacobian,c.nnz*2*sizeof(double),cudaMemcpyDeviceToHost));
    return 0;
  } catch (const std::exception& error) { last_error=error.what(); return 1; }
}
}  // extern C
