#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace ps {
// IEEE reflected CRC32. Shared constexpr tables; no target-specific CRC ISA.
// Explicit little-endian byte assembly permits unaligned inputs on any host.
inline constexpr auto crc_tables = [] {
    std::array<std::array<uint32_t,256>,8> tables{};
    for(uint32_t i=0;i<256;++i){
        uint32_t c=i;
        for(int j=0;j<8;++j)c=(c>>1)^((c&1)?0xedb88320u:0);
        tables[0][i]=c;
    }
    for(int s=1;s<8;++s)for(int i=0;i<256;++i){
        const uint32_t c=tables[s-1][i];
        tables[s][i]=tables[0][c&255]^(c>>8);
    }
    return tables;
}();
inline uint32_t crc32_update(uint32_t state,const unsigned char* p,size_t n){
    while(n>=8){
        const uint32_t a=state^uint32_t(p[0])^(uint32_t(p[1])<<8)
                            ^(uint32_t(p[2])<<16)^(uint32_t(p[3])<<24);
        state=crc_tables[7][a&255]^crc_tables[6][(a>>8)&255]
             ^crc_tables[5][(a>>16)&255]^crc_tables[4][a>>24]
             ^crc_tables[3][p[4]]^crc_tables[2][p[5]]
             ^crc_tables[1][p[6]]^crc_tables[0][p[7]];
        p+=8;n-=8;
    }
    while(n--){state=crc_tables[0][(state^*p++)&255]^(state>>8);}
    return state;
}
} // namespace ps
