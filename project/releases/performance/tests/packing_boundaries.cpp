#include "../src/row_pair.hpp"
#include <cstring>
#include <iostream>
int main(){
    try{
        alignas(16) unsigned char buffer[96];std::memset(buffer,0xa5,sizeof buffer);
        std::vector<double> a={-0.,1.,-2.,3.,4.,5.,6.,7.,8.};
        auto p=ps_layout::pack(a,3);if(p.size()!=6)throw std::runtime_error("Odd word count");
        if(std::memcmp(&p[0].lo,&a[0],8))throw std::runtime_error("Signed zero changed");
        unsigned rejects=0;
        auto test=[&](const double* src,size_t count,ps_layout::Pair* dst,size_t capacity,unsigned n){
            try{ps_layout::pack_into(src,count,dst,capacity,n);}catch(const std::invalid_argument&){++rejects;}
        };
        test(a.data(),9,reinterpret_cast<ps_layout::Pair*>(buffer+8),6,3);
        test(a.data(),9,nullptr,6,3);test(nullptr,9,p.data(),6,3);
        test(a.data(),8,p.data(),6,3);test(a.data(),9,p.data(),5,3);test(a.data(),9,p.data(),7,3);
        test(a.data(),9,p.data(),6,0);test(a.data(),9,p.data(),6,1045);
        if(rejects!=8)throw std::runtime_error("Bad buffer accepted");
        for(unsigned char c:buffer)if(c!=0xa5)throw std::runtime_error("Rejected buffer modified");
        if(ps_layout::words(1043)*16!=8711136||ps_layout::words(1044)*16!=8719488)throw std::runtime_error("Slot sizing");
        std::cout<<"PACKING_BOUNDARIES_PASS rejected=8 signed_zero=exact"<<std::endl;
    }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
}
