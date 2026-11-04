// ============================================================================
// 位宽契约探针（X13 双口径用）
//
// 编译期 -D 决定档位，运行时打印各节点位宽，供验收脚本与 X1 第六节的公式对照：
//   bs  = PS_IN_BITS + PS_BS_EXTRA + PS_GS_EXTRA  （块和内像素数的对数折进 EXTRA；
//                                                   +1 是双绿 G1+G2 那一位，见 X29/X27）
//   acc = bs + PS_ACC_EXTRA                 （帧累积）
//   dif = acc + 1                           （符号位）
//   gsum= PS_IN_BITS + PS_GS_EXTRA          （双绿求和，P2：不求平均）
//
// 用法：
//   g++ -O2 -std=c++11 bitwidth_probe.cpp -o bw
//   ./bw           → 期望 in=16 bs=24 acc=28 dif=29 gsum=17
//   g++ -O2 -std=c++11 -DPS_IN_BITS=12 -DPS_BLOCK=8 -DPS_FRAMES=1 \
//       -DPS_BS_EXTRA=6 -DPS_ACC_EXTRA=0 bitwidth_probe.cpp -o bw12
//   ./bw12        → 期望 in=12 bs=19 acc=19 dif=20 gsum=13
// （12 bit 那档的旧期望是 bs=18 acc=18 dif=19 —— 那是**没算双绿 +1 bit**时的值；
//   加宽后一律 +1，实测见 sim/results/x29_config_consistency.md。）
// ============================================================================
#include <cstdio>

#include "preprocess.hpp"

int main() {
    std::printf("in=%d bs=%d acc=%d dif=%d gsum=%d block=%d frames=%d color=%d\n",
                PS_IN_BITS, PS_BS_BITS, PS_ACC_BITS, PS_DIF_BITS, PS_GS_BITS,
                PS_BLOCK, PS_FRAMES, PS_COLOR);
    return 0;
}
