#pragma once
#include <cstddef>
#include <cstdint>
namespace ps_layout {
constexpr unsigned MAX_N=1044;
constexpr unsigned VERSION=0x32525001u;
constexpr unsigned OK=0x46500200u, BAD_N=0x46500201u, BAD_LAYOUT=0x46500202u;
constexpr unsigned REG_MATRIX=0x10,REG_INPUT=0x1c,REG_OUTPUT=0x28;
constexpr unsigned REG_N=0x34,REG_STATUS=0x3c,REG_LAYOUT=0x44;
constexpr char RELEASE[]="20261015-rp2-v1";
struct alignas(16) Pair {double lo;double hi;};
static_assert(sizeof(Pair)==16 && alignof(Pair)==16,"128-bit matrix word required");
static_assert(offsetof(Pair,hi)==8,"Lane order is part of the ABI");
constexpr size_t words(unsigned n){return size_t((n+1)/2)*n;}
}
extern "C" unsigned full_gemv(const ps_layout::Pair*,const double*,double*,unsigned,unsigned);
