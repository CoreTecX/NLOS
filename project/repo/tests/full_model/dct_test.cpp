#include "../src/dct.hpp"
#include "../reference/dct_original.hpp"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <cstdint>

constexpr int H=29,W=36,N=H*W,S=N*3;
using Vec=std::vector<double>;
using Candidate=ps::SeparableRInverse<H,W>;
static void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
static uint64_t rng=0x143574ULL;
static double random_value(){rng^=rng<<13;rng^=rng>>7;rng^=rng<<17;return double(rng>>11)/9007199254740992.*2.-1.;}

__attribute__((noinline)) void original_run(frozen::RInverse& d,const Vec& a,const Vec& s,Vec& y){d.run(a,s,y);}
__attribute__((noinline)) void candidate_run(Candidate& d,const Vec& a,const Vec& s,Vec& y){d.run(a,s,y);}

int main(int argc,char** argv){try{
    Candidate optimized;frozen::RInverse original;
    Vec spec(N),a(S),expected(S),actual(S+6,123456.25);
    const double pi=std::acos(-1.);
    for(int k=0;k<H;++k)for(int j=0;j<W;++j)
        spec[k*W+j]=1.+4.*std::pow(std::sin(pi*k/(2.*H)),2)+4.*std::pow(std::sin(pi*j/(2.*W)),2);
    int cases=0;double worst_residual=0;
    auto check=[&](bool residual){
        original_run(original,a,spec,expected);candidate_run(optimized,a,spec,actual);
        require(std::memcmp(actual.data(),expected.data(),S*sizeof(double))==0,"DCT differs from frozen scalar reduction");
        for(int i=S;i<S+6;++i)require(actual[i]==123456.25,"DCT output overrun");
        if(residual){double peak=0,err=0;
            for(int r=0;r<H;++r)for(int c=0;c<W;++c)for(int z=0;z<3;++z){
                int i=(r*W+c)*3+z;double v=actual[i];
                if(r)v+=actual[i]-actual[i-W*3];
                if(r+1<H)v+=actual[i]-actual[i+W*3];
                if(c)v+=actual[i]-actual[i-3];
                if(c+1<W)v+=actual[i]-actual[i+3];
                err=std::max(err,std::abs(v-a[i]));peak=std::max(peak,std::abs(a[i]));
            }
            double rel=err/std::max(peak,1e-300);worst_residual=std::max(worst_residual,rel);
            require(rel<1e-12,"Independent (I + Neumann Laplacian) residual failed");
        }
        ++cases;
    };
    if(argc==2&&std::string(argv[1])=="bench"){
        for(double& x:a)x=random_value();
        volatile double checksum=0;
        auto sample=[&](bool opt,int count){
            const auto begin=std::chrono::steady_clock::now();
            for(int i=0;i<count;++i){a[i%S]+=(i%2?1e-6:-1e-6);
                if(opt)candidate_run(optimized,a,spec,actual);else original_run(original,a,spec,actual);
                checksum+=actual[(i*17)%S];}
            return std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()/count;
        };
        sample(false,100);sample(true,100);
        std::vector<double> old_times,new_times;
        for(int i=0;i<9;++i){if(i%2){new_times.push_back(sample(true,1500));old_times.push_back(sample(false,1500));}
            else{old_times.push_back(sample(false,1500));new_times.push_back(sample(true,1500));}}
        std::cout<<std::setprecision(17)<<"{\"uses_neon\":"<<(Candidate::uses_neon?"true":"false")<<",\"calls_per_sample\":1500,\"original_seconds_per_call\":[";
        for(size_t i=0;i<old_times.size();++i){if(i)std::cout<<',';std::cout<<old_times[i];}
        std::cout<<"],\"candidate_seconds_per_call\":[";
        for(size_t i=0;i<new_times.size();++i){if(i)std::cout<<',';std::cout<<new_times[i];}
        std::cout<<"],\"checksum\":"<<checksum<<"}\n";return 0;
    }
    check(true); // zero and signed/zero accumulation behavior
    for(int index=0;index<S;++index){std::fill(a.begin(),a.end(),0);a[index]=1.;check(true);}
    for(double scale:{1e-100,1.,1e100}){
        for(int trial=0;trial<16;++trial){for(double& x:a)x=scale*random_value();check(true);}
        std::fill(a.begin(),a.end(),scale);check(true);
        for(int i=0;i<S;++i)a[i]=(i%2?scale:-scale);
        check(true);
    }
    // Distinct spectra, channel isolation and repeated reuse detect stale state.
    for(int trial=0;trial<16;++trial){
        for(int i=0;i<S;++i)a[i]=i%3==trial%3?random_value():0;
        for(double& x:spec)x=1.+std::abs(random_value())*1000.;
        check(false);
        Vec alias=a;candidate_run(optimized,alias,spec,alias);
        require(std::memcmp(alias.data(),expected.data(),S*sizeof(double))==0,"Aliased input/output mismatch");++cases;
    }
    std::fill(spec.begin(),spec.end(),1.);for(double& x:a)x=random_value();check(false);
    double identity=0;for(int i=0;i<S;++i)identity=std::max(identity,std::abs(actual[i]-a[i]));
    require(identity<1e-13,"Identity spectrum round trip failed");
    std::cout<<std::setprecision(17)<<"{\"passed\":true,\"cases\":"<<cases<<",\"bit_exact\":true,\"uses_neon\":"
             <<(Candidate::uses_neon?"true":"false")<<",\"worst_operator_relative_peak_residual\":"<<worst_residual
             <<",\"identity_max_absolute_error\":"<<identity<<"}\n";
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
