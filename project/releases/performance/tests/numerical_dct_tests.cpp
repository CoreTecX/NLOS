#include "../src/full_runtime.hpp"
#include "experimental/mixed_dct.hpp"
#include "experimental/fma_dct.hpp"
#include <random>
template<class T> double time_kernel(T& kernel,const ps::Vec& x,const ps::Vec& spec,ps::Vec& y){
    const auto start=std::chrono::steady_clock::now();
    for(int i=0;i<100;++i)kernel.run(x,spec,y);
    ps::finite(y);return std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()/100.;
}
int main(){try{
    ps::RInverse direct;ps::MixedRInverse<29,36> mixed;ps::FmaRInverse<29,36> fma;
    ps::Vec x(ps::S),spec(ps::N),a(ps::S),b(ps::S),c(ps::S);
    std::mt19937_64 rng(789);std::uniform_real_distribution<double> random(-1.,1.);
    for(int r=0;r<29;++r)for(int col=0;col<36;++col)spec[r*36+col]=5.-2.*std::cos(std::acos(-1.)*r/29)-2.*std::cos(std::acos(-1.)*col/36);
    double mixed_error=0,fma_error=0;
    for(int test=0;test<3220;++test){
        if(test<ps::S){std::fill(x.begin(),x.end(),0.);x[test]=1.;}else for(auto& v:x)v=random(rng);
        direct.run(x,spec,a);mixed.run(x,spec,b);fma.run(x,spec,c);ps::finite(b);ps::finite(c);
        double norm=0,em=0,ef=0;for(int i=0;i<ps::S;++i){norm+=a[i]*a[i];em+=(a[i]-b[i])*(a[i]-b[i]);ef+=(a[i]-c[i])*(a[i]-c[i]);}
        mixed_error=std::max(mixed_error,std::sqrt(em/norm));fma_error=std::max(fma_error,std::sqrt(ef/norm));
    }
    // Aliasing and repeated workspace reuse.
    mixed.run(x,spec,b);c=x;mixed.run(c,spec,c);ps::require(b==c,"Mixed aliasing failed");
    fma.run(x,spec,b);c=x;fma.run(c,spec,c);ps::require(b==c,"FMA aliasing failed");
    ps::require(mixed_error<1e-11&&fma_error<1e-11,"Numerical DCT error gate");
    time_kernel(direct,x,spec,a);time_kernel(mixed,x,spec,b);time_kernel(fma,x,spec,c);
    double td[5],tm[5],tf[5];for(int i=0;i<5;++i){
        if(i%2){tf[i]=time_kernel(fma,x,spec,c);tm[i]=time_kernel(mixed,x,spec,b);td[i]=time_kernel(direct,x,spec,a);}
        else{td[i]=time_kernel(direct,x,spec,a);tm[i]=time_kernel(mixed,x,spec,b);tf[i]=time_kernel(fma,x,spec,c);}
    }
    std::cout<<std::setprecision(17)<<"{\"passed\":true,\"cases\":3220,\"bit_exact_claim\":false,\"mixed_relative_l2\":"<<mixed_error<<",\"fma_relative_l2\":"<<fma_error<<",\"seconds_per_run\":{";
    auto list=[](const char* key,const double* values){std::cout<<'"'<<key<<"\":[";for(int i=0;i<5;++i){if(i)std::cout<<',';std::cout<<values[i];}std::cout<<']';};
    list("direct",td);std::cout<<',';list("mixed",tm);std::cout<<',';list("fma",tf);std::cout<<"}}\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
