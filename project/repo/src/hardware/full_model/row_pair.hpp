#pragma once
#include "gemv_contract.hpp"
#include <stdexcept>
#include <vector>
namespace ps_layout {
// dst: 16-byte aligned; source: n*n row-major FP64 values. Buffers must not overlap.
inline void pack_into(const double* src,size_t src_count,Pair* dst,size_t dst_count,unsigned n){
    if(n==0 || n>MAX_N || !src || !dst || src_count!=size_t(n)*n || dst_count!=words(n)
       || (reinterpret_cast<uintptr_t>(dst)&15u))throw std::invalid_argument("Invalid row-pair buffer");
    for(unsigned r=0;r<n;r+=2)for(unsigned j=0;j<n;++j){
        Pair& p=dst[size_t(r/2)*n+j];p.lo=src[size_t(r)*n+j];
        p.hi=r+1<n?src[size_t(r+1)*n+j]:0.;
    }
}
inline std::vector<Pair> pack(const std::vector<double>& src,unsigned n){
    if(n==0 || n>MAX_N)throw std::invalid_argument("Invalid row-pair dimension");
    std::vector<Pair> dst(words(n));pack_into(src.data(),src.size(),dst.data(),dst.size(),n);return dst;
}
}
