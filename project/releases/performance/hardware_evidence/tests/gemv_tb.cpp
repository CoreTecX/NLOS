#include "../src/row_pair.hpp"
#include <cstring>
#include <iostream>
#include <random>
extern "C" unsigned reference_gemv(const double*,const double*,double*,unsigned);
void check(bool ok,const char* msg){if(!ok)throw std::runtime_error(msg);}
int main(){
    try{
        check(full_gemv(nullptr,nullptr,nullptr,1044,0)==ps_layout::BAD_LAYOUT,"Wrong layout guard");
        for(unsigned n:{0u,1045u,0xffffffffu})check(full_gemv(nullptr,nullptr,nullptr,n,ps_layout::VERSION)==ps_layout::BAD_N,"Dimension guard");
        std::mt19937_64 rng(20261015);std::uniform_real_distribution<double> dist(-1.,1.);
        for(unsigned n:{1u,2u,15u,16u,17u,31u,32u,33u,63u,64u,65u,1043u,1044u}){
            std::vector<double> a(size_t(n)*n),x(3*n),ref(3*n),y(3*n+16,12345.);
            for(double& v:a)v=dist(rng);
            for(double& v:x)v=dist(rng);
            auto p=ps_layout::pack(a,n);
            check((reinterpret_cast<uintptr_t>(p.data())&15u)==0,"Alignment");
            for(unsigned r=0;r<n;++r)for(unsigned j=0;j<n;++j){
                const double v=(r&1)?p[size_t(r/2)*n+j].hi:p[size_t(r/2)*n+j].lo;
                check(std::memcmp(&v,&a[size_t(r)*n+j],8)==0,"Pack/unpack mismatch");
            }
            if(n&1)for(unsigned j=0;j<n;++j)check(p[size_t(n/2)*n+j].hi==0.,"Odd padding");
            check(reference_gemv(a.data(),x.data(),ref.data(),n)==0x46500100u,"Reference");
            check(full_gemv(p.data(),x.data(),y.data(),n,ps_layout::VERSION)==ps_layout::OK,"New kernel");
            check(std::memcmp(ref.data(),y.data(),3*n*8)==0,"FP64 bit mismatch");
            for(size_t i=3*n;i<y.size();++i)check(y[i]==12345.,"Output overrun");
            if(n&1){for(unsigned j=0;j<n;++j)p[size_t(n/2)*n+j].hi=9876.;
                full_gemv(p.data(),x.data(),y.data(),n,ps_layout::VERSION);
                check(std::memcmp(ref.data(),y.data(),3*n*8)==0,"Padded lane contaminated rows");
                for(size_t i=3*n;i<y.size();++i)check(y[i]==12345.,"Padded output overrun");}
            for(unsigned repeat=0;repeat<2;++repeat){
                check(full_gemv(nullptr,nullptr,nullptr,0,ps_layout::VERSION)==ps_layout::BAD_N,"Interleaved invalid request");
                for(double& v:x)v=dist(rng);
                check(reference_gemv(a.data(),x.data(),ref.data(),n)==0x46500100u,"Reuse reference");
                check(full_gemv(p.data(),x.data(),y.data(),n,ps_layout::VERSION)==ps_layout::OK,"Reuse kernel");
                check(std::memcmp(ref.data(),y.data(),3*n*8)==0,"Stale partial or input on reuse");
                for(size_t i=3*n;i<y.size();++i)check(y[i]==12345.,"Reuse output overrun");
            }
            std::cout<<"GEMV_BIT_EXACT n="<<n<<" words="<<p.size()<<std::endl;
        }
        int rejects=0;for(unsigned n:{0u,1045u,0xffffffffu})try{ps_layout::pack({},n);}catch(const std::invalid_argument&){++rejects;}
        try{ps_layout::pack({1.},2);}catch(const std::invalid_argument&){++rejects;}
        check(rejects==4,"Packing guards");
        std::cout<<"P56_NATIVE_PASS sizes=13 reuse_calls=26 rejected_requests=34"<<std::endl;
    }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
}
