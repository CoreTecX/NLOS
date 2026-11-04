#include "../src/full_runtime.hpp"
#include "packed_host.hpp"
#include <numeric>

static uint32_t bit_crc(const unsigned char* p,size_t n){
    uint32_t c=~0u;
    for(size_t i=0;i<n;++i){c^=p[i];for(int b=0;b<8;++b)c=(c>>1)^((c&1)?0xedb88320u:0);}
    return ~c;
}
static bool bits(double a,double b){return std::memcmp(&a,&b,8)==0;}
static double sorted_percentile(ps::Vec a,double f){
    std::sort(a.begin(),a.end());const double at=f*(a.size()-1);size_t i=size_t(at);double w=at-i;
    return a[i]+w*(a[std::min(i+1,a.size()-1)]-a[i]);
}
template<class F> static void rejected(F&& f,const char* message){
    bool ok=false;try{f();}catch(const std::runtime_error& e){ok=std::string(e.what())==message;}
    ps::require(ok,"Missing rejection or changed error");
}
int main(int argc,char** argv){try{
    using ps::require;
    require(argc==1||argc==3,"runtime_tests [MODEL MEASUREMENT]");
    std::vector<unsigned char> data(10017);uint32_t rng=0x12345678;
    for(auto& b:data){rng=rng*1664525+1013904223;b=static_cast<unsigned char>(rng>>24);}
    size_t crc_cases=0,percentile_cases=0,median_cases=0;
    const unsigned char known[]="123456789";
    require(ps::crc32(known,9)==0xcbf43926u,"CRC known vector");
    for(size_t offset=0;offset<16;++offset)for(size_t n:std::initializer_list<size_t>{size_t(0),1u,2u,7u,8u,9u,15u,16u,17u,255u,256u,4095u,4096u,10000u}){
        const auto* p=data.data()+offset;const auto expected=bit_crc(p,n);
        require(ps::crc32(p,n)==expected,"CRC bit oracle");
        for(size_t chunk:std::initializer_list<size_t>{size_t(1),3u,8u,17u,4096u}){
            uint32_t c=~0u;for(size_t pos=0;pos<n;pos+=chunk)c=ps::crc32_update(c,p+pos,std::min(chunk,n-pos));
            require(~c==expected,"CRC stream boundary");++crc_cases;
        }
    }
    for(size_t n:std::initializer_list<size_t>{size_t(1),2u,3u,9u,16u,127u,1000u,15625u,47628u}){
        for(int mode=0;mode<8;++mode){ps::Vec a(n);
            for(size_t i=0;i<n;++i){rng=rng*1664525+1013904223;
                a[i]=mode==0?double(i):mode==1?double(n-i):mode==2?1.:mode==3?double(int(i%7)-3):double(int(rng%65536)-32768)/8192.;
                if(mode==5&&i%3==0)a[i]=-0.;
                if(mode==6)a[i]*=1e100;
                if(mode==7)a[i]*=1e-100;
            }
            for(double f:{0.,.01,.5,.99,1.}){require(bits(ps::percentile(a,f),sorted_percentile(a,f)),"Percentile differs from full sort");++percentile_cases;}
        }
    }
    rejected([]{ps::percentile({},.5);},"Empty percentile input");
    double a[9];std::iota(a,a+9,0.);
    do{double b[9];std::copy(a,a+9,b);require(ps::median9(b)==4.,"Median permutation");++median_cases;}while(std::next_permutation(a,a+9));
    for(int trial=0;trial<10000;++trial){double b[9];for(int i=0;i<9;++i){rng=rng*1664525+1013904223;a[i]=double(int(rng%7)-3);b[i]=a[i];}
        std::sort(a,a+9);require(bits(ps::median9(b),a[4]),"Median ties");++median_cases;}
    bool lifetime=false;
    if(argc==3){
        ps::Model m(argv[1]);ps::Measurement meas(argv[2],m);ps::Host cpu(m);
        ps::Vec x(ps::S),expected(ps::S),actual(ps::S);for(int i=0;i<ps::S;++i)x[i]=double((i%17)-8)/8.;
        const size_t basis=m.q.capacity()*sizeof(int16_t);m.release_measurement_basis();
        require(basis==size_t(ps::N)*ps::M*2&&m.q.capacity()==0,"Basis not released");
        rejected([&]{ps::Measurement again(argv[2],m);},"Measurement basis released: reload model before another measurement");
        ps::PackedHost packed(m);
        for(bool t:{false,true}){cpu.multiply(t,x,expected);packed.multiply(t,x,actual);require(std::memcmp(expected.data(),actual.data(),ps::S*8)==0,"Packed initial state");}
        cpu.multiply(false,x,expected);m.release_cpu_matrices();
        require(m.k.capacity()==0&&m.kt.capacity()==0,"Matrices not released");
        packed.multiply(false,x,actual);require(std::memcmp(expected.data(),actual.data(),ps::S*8)==0,"Packed owns its matrices");
        rejected([&]{cpu.multiply(false,x,actual);},"CPU matrices released: reload model before CPU fallback");
        rejected([&]{ps::Host invalid(m);},"CPU matrices released: reload model before CPU fallback");
        rejected([&]{ps::PackedHost invalid(m);},"CPU matrices released: reload model before CPU fallback");
        ps::Model reload(argv[1]);ps::Host fallback(reload);fallback.multiply(false,x,actual);
        require(std::memcmp(expected.data(),actual.data(),ps::S*8)==0,"Reloaded CPU fallback mismatch");
        lifetime=true;
    }
    std::cout<<"{\"passed\":true,\"crc_cases\":"<<crc_cases<<",\"percentile_cases\":"<<percentile_cases<<",\"median_cases\":"<<median_cases<<",\"lifetime_checked\":"<<(lifetime?"true":"false")<<"}\n";
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
