#pragma once
#include "../src/full_runtime.hpp"
#include "../src/row_pair.hpp"
namespace ps {
// Test-only backend: caches both layouts once, runs the identical new HLS C code.
struct PackedHost:Backend {
    std::vector<ps_layout::Pair> k,kt;
    explicit PackedHost(const Model& m){m.require_cpu_matrices();k=ps_layout::pack(m.k,N);kt=ps_layout::pack(m.kt,N);}
    void multiply(bool transpose,const Vec& x,Vec& y)override{
        require(full_gemv((transpose?kt:k).data(),x.data(),y.data(),N,ps_layout::VERSION)==ps_layout::OK,"Packed host failure");
    }
};
}
