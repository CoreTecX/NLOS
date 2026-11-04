#pragma once
#include "dct.hpp"
#include "crc32.hpp"
#include "phase_profile.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
namespace ps {
constexpr int H=29,W=36,N=H*W,M=126*126,S=N*3;
using Vec=std::vector<double>;
inline void require(bool b,const char* s){if(!b)throw std::runtime_error(s);}
inline uint32_t crc32(const unsigned char* p,size_t n){
    PS_SCOPE(profile_crc, InputCRC);
    return ~crc32_update(~0u,p,n);
}
template<class T> T get(const unsigned char* p){T x;std::memcpy(&x,p,sizeof(T));return x;}
// Stream directly into final arrays. The complete payload (including unused F)
// remains CRC-checked before Model/Measurement construction can succeed.
struct Blob {
    std::array<unsigned char,16> id;
    std::ifstream file;
    size_t remaining;
    uint32_t expected_crc=0,state=~0u;
    Blob(const std::string& path,unsigned kind,unsigned rows,unsigned cols,size_t expected)
        :file(path,std::ios::binary|std::ios::ate),remaining(expected){
        const uint16_t e=1;require(*reinterpret_cast<const unsigned char*>(&e)==1,"Little-endian target required");
        require(bool(file),"Cannot open input");
        require(file.tellg()==std::streamoff(64+expected),"Wrong input byte count");file.seekg(0);
        unsigned char hdr[64];file.read(reinterpret_cast<char*>(hdr),64);require(bool(file),"Truncated header");
        require(!std::memcmp(hdr,"PSFULL1\0",8)&&get<uint32_t>(hdr+8)==1,"Wrong file magic/version");
        require(get<uint32_t>(hdr+12)==kind&&get<uint32_t>(hdr+16)==rows&&get<uint32_t>(hdr+20)==cols,"Wrong kind/dimensions");
        require(get<uint64_t>(hdr+24)==expected&&get<uint32_t>(hdr+36)==0&&get<uint64_t>(hdr+56)==0,"Invalid header");
        std::copy(hdr+40,hdr+56,id.begin());expected_crc=get<uint32_t>(hdr+32);
    }
    void read_bytes(void* destination,size_t bytes){
        require(bytes<=remaining,"Truncated payload");
        PS_SCOPE(profile_io, InputRead);
        file.read(static_cast<char*>(destination),std::streamsize(bytes));require(bool(file),"Truncated payload");
        PS_NEXT(profile_io, InputCRC);
        const auto* p=static_cast<const unsigned char*>(destination);
        state=crc32_update(state,p,bytes);
        remaining-=bytes;
    }
    template<class T>std::vector<T> take(size_t n){
        require(n<=remaining/sizeof(T),"Truncated payload");
        std::vector<T> a(n);read_bytes(a.data(),n*sizeof(T));return a;
    }
    void discard_finite_doubles(size_t n){
        require(n<=remaining/sizeof(double),"Truncated payload");
        std::array<double,4096> chunk{};
        while(n){const size_t count=std::min(n,chunk.size());read_bytes(chunk.data(),count*sizeof(double));
            for(size_t i=0;i<count;++i)require(std::isfinite(chunk[i]),"Nonfinite input/state");
            n-=count;
        }
    }
    void finish(){
        require(remaining==0,"Unconsumed payload");require(~state==expected_crc,"Input CRC mismatch");
        require(file.peek()==std::char_traits<char>::eof()&&!file.bad(),"Trailing input data/read error");
    }
};
inline void finite(const Vec& a){for(double x:a)require(std::isfinite(x),"Nonfinite input/state");}
// Negative zero ties retain the original sort path and interpolation semantics.
inline double percentile(Vec a,double fraction){
    require(!a.empty(),"Empty percentile input");
    const double at=fraction*(a.size()-1);size_t i=size_t(at);double w=at-i;
    if(std::any_of(a.begin(),a.end(),[](double x){return x==0.&&std::signbit(x);})){
        std::sort(a.begin(),a.end());
        return a[i]+w*(a[std::min(i+1,a.size()-1)]-a[i]);
    }
    std::nth_element(a.begin(),a.begin()+i,a.end());
    const double lo=a[i];
    const double hi=i+1<a.size()?*std::min_element(a.begin()+i+1,a.end()):lo;
    return lo+w*(hi-lo);
}
inline double median9(double (&a)[9]){
    // Fixed comparison network, only the middle value is consumed.
    auto order=[&](int i,int j){if(a[j]<a[i])std::swap(a[i],a[j]);};
    order(1,2);order(4,5);order(7,8);order(0,1);order(3,4);order(6,7);
    order(1,2);order(4,5);order(7,8);order(0,3);order(5,8);order(4,7);
    order(3,6);order(1,4);order(2,5);order(4,7);order(4,2);order(6,4);order(4,2);
    return a[4];
}

struct Model {
    std::array<unsigned char,16> id;double norm;Vec scales,k,kt,ev,rspec;std::vector<int16_t> q;
    // Single-measurement CLI releases q after h is complete. Loading another
    // measurement then requires a fresh Model; CPU K/KT remain independently owned.
    void release_measurement_basis(){std::vector<int16_t>().swap(q);}
    // Only call after a backend has successfully acquired its own matrix storage.
    // CPU fallback after this point requires reloading a Model, never stale pointers.
    void release_cpu_matrices(){Vec().swap(k);Vec().swap(kt);}
    void require_cpu_matrices()const{require(k.size()==size_t(N)*N&&kt.size()==size_t(N)*N,"CPU matrices released: reload model before CPU fallback");}
    explicit Model(const std::string& path){
        Blob b(path,1,H,W,8+N*8+size_t(N)*M*2+size_t(N)*N*8*3+N*16);id=b.id;
        norm=b.take<double>(1)[0];scales=b.take<double>(N);q=b.take<int16_t>(size_t(N)*M);
        k=b.take<double>(N*N);kt=b.take<double>(N*N);ev=b.take<double>(N);rspec=b.take<double>(N);
        b.discard_finite_doubles(size_t(N)*N);b.finish();
        require(norm>0&&std::isfinite(norm),"Invalid model norm");finite(scales);finite(k);finite(kt);finite(ev);finite(rspec);
        for(int i=0;i<N;++i){require(scales[i]>0&&ev[i]>=0&&rspec[i]>=1,"Invalid model scale/spectrum");
            for(int j=0;j<N;++j)require(k[i*N+j]==kt[j*N+i],"Transpose mismatch");}
    }
};
struct Measurement {
    double ds=0,multiplier=0;Vec cc,h;
    double gradient_relative_norm=0,gradient_rms=0,noise_proxy=0,outlier_ratio=0;
    std::array<double,3> noise{};std::vector<std::string> flags;
    Measurement(const std::string& path,const Model& m):cc(3),h(S,0){
        Blob b(path,2,126,126,24+M*3*8);require(b.id==m.id,"Measurement/model identity mismatch");
        Vec scales=b.take<double>(3),raw=b.take<double>(M*3);b.finish();finite(scales);finite(raw);
        require(m.q.size()==size_t(N)*M,"Measurement basis released: reload model before another measurement");
        for(double x:scales)require(x>0,"RGB scales must be positive");
        for(double x:raw)require(x>=0,"Raw measurement must be nonnegative");
        PS_SCOPE(profile_meas, InputDiagnostics);
        const double mx=*std::max_element(scales.begin(),scales.end());
        for(int c=0;c<3;++c){cc[c]=scales[c]/mx;require(cc[c]>0,"Scale underflow");}
        for(int p=0;p<M;++p)for(int c=0;c<3;++c)ds=std::max(ds,std::abs(p==M-1?raw[3*p+c]:raw[3*p+c]-raw[3*(p+1)+c]));
        require(ds>0&&std::isfinite(ds),"Empty/overflowed measurement");
        double yn=0,dn=0;for(double x:raw)yn+=x*x;
        for(int p=0;p<M;++p)for(int c=0;c<3;++c){
            if(p%126!=125){double d=raw[p*3+c]-raw[(p+1)*3+c];dn+=d*d*.5;}
            if(p+126<M){double d=raw[p*3+c]-raw[(p+126)*3+c];dn+=d*d*.5;}}
        require(std::isfinite(yn)&&std::isfinite(dn)&&std::sqrt(dn)>1e-12*std::max(1.,std::sqrt(yn)),"No useful spatial variation");
        gradient_relative_norm=std::sqrt(dn/yn);gradient_rms=std::sqrt(dn/(31500*3));
        Vec local;local.reserve(raw.size());double cn[3]={};
        for(int c=0;c<3;++c){Vec cross;cross.reserve(125*125);
            for(int col=1;col<126;++col)for(int row=1;row<126;++row){int i=(row+126*col)*3+c;
                cross.push_back(raw[i]-raw[i-126*3]-raw[i-3]+raw[i-127*3]);}
            double center=percentile(cross,.5);for(double& x:cross)x=std::abs(x-center);
            noise[c]=percentile(cross,.5)/(2*.67448975);
            for(int col=0;col<126;++col)for(int row=0;row<126;++row){
                double neighbors[9];int k=0;
                for(int dc=-1;dc<=1;++dc)for(int dr=-1;dr<=1;++dr){int rr=std::max(0,std::min(125,row+dr)),cl=std::max(0,std::min(125,col+dc));neighbors[k++]=raw[(rr+126*cl)*3+c];}
                const double med=median9(neighbors);double v=raw[(row+126*col)*3+c];cn[c]+=v*v;local.push_back(std::abs(v-med));
            }
        }
        noise_proxy=std::min(1e30,gradient_rms/std::max(std::sqrt((noise[0]*noise[0]+noise[1]*noise[1]+noise[2]*noise[2])/3),1e-30));
        outlier_ratio=std::min(1e30,*std::max_element(local.begin(),local.end())/std::max(percentile(local,.99),1e-30));
        if(noise_proxy<3)flags.push_back("low_gradient_to_noise_proxy");
        if(outlier_ratio>30)flags.push_back("isolated_pixel_outlier");
        const double cm=std::sqrt(*std::max_element(cn,cn+3));
        for(double x:cn)if(std::sqrt(x)<1e-10*std::max(cm,1e-30)){flags.push_back("empty_rgb_channel");break;}
        PS_NEXT(profile_meas, HConstruct);
        const double root2=std::sqrt(2.);
        // Sum (model[p]-model[q])*d[p,q] as model dot B^T d.
        // B uses only actual image edges: no wraparound, padding or boundary edge.
        // This changes FP64 accumulation order; validate against the frozen h/output.
        Vec adjoint_measurement(M*3,0);
        for(int p=0;p<M;++p)if(p%126!=125){
            for(int c=0;c<3;++c){
                const double d=(raw[3*p+c]-raw[3*(p+1)+c])/root2/ds;
                adjoint_measurement[3*p+c]+=d;adjoint_measurement[3*(p+1)+c]-=d;
            }
        }
        for(int p=0;p<M-126;++p){
            for(int c=0;c<3;++c){
                const double d=(raw[3*p+c]-raw[3*(p+126)+c])/root2/ds;
                adjoint_measurement[3*p+c]+=d;adjoint_measurement[3*(p+126)+c]-=d;
            }
        }
        for(int i=0;i<N;++i){
            const int16_t* q=m.q.data()+size_t(i)*M;double sum[3]={};
            for(int p=0;p<M;++p){
                const double a=double(q[p])*m.scales[i]/root2/m.norm;
                for(int c=0;c<3;++c)sum[c]+=a*adjoint_measurement[3*p+c];
            }
            for(int c=0;c<3;++c)h[3*i+c]=sum[c]*cc[c];
        }
        multiplier=ds/(m.norm*mx);require(multiplier>0&&std::isfinite(multiplier),"Physical scale invalid");finite(h);
    }
};
inline double norm(const Vec& a){double s=0;for(double x:a)s+=x*x;return std::sqrt(s);}
inline void grad(const Vec& a,Vec& g){
    std::fill(g.begin(),g.end(),0);
    for(int r=0;r<H;++r)for(int c=0;c<W;++c)for(int z=0;z<3;++z){int i=(r*W+c)*3+z;
        if(r+1<H)g[i]=a[i+W*3]-a[i];
        if(c+1<W)g[S+i]=a[i+3]-a[i];}
}
inline void adj(const Vec& g,Vec& a){
    std::fill(a.begin(),a.end(),0);
    for(int r=0;r<H-1;++r)for(int c=0;c<W*3;++c){int i=r*W*3+c;a[i]-=g[i];a[i+W*3]+=g[i];}
    for(int r=0;r<H;++r)for(int c=0;c<(W-1)*3;++c){int i=r*W*3+c;a[i]-=g[S+i];a[i+3]+=g[S+i];}
}
#ifdef PS_DCT_LEGACY
#include "dct_legacy.inc"
#else
using RInverse = SeparableRInverse<H,W>;
#endif
struct Backend {virtual void multiply(bool transpose,const Vec& x,Vec& y)=0;virtual ~Backend()=default;};
extern "C" unsigned reference_gemv(const double*,const double*,double*,unsigned);
struct Host:Backend {const Model& m;explicit Host(const Model& v):m(v){m.require_cpu_matrices();}
    void multiply(bool transpose,const Vec& x,Vec& y)override{
        m.require_cpu_matrices();
        require(reference_gemv((transpose?m.kt:m.k).data(),x.data(),y.data(),N)==0x46500100u,"Host kernel failed");
    }
};
struct Result {Vec f;int iterations=0;bool passed=false;double rho=.01,primal=0,dual=0,plim=0,dlim=0,seconds=0;
    std::vector<std::array<double,6>> trace;
};
inline Result solve(const Model& m,const Measurement& meas,Backend& backend,int limit=20000){
    PS_SCOPE(profile_solver, Solver);
    PS_SCOPE(profile_setup, SolverSetup);
    const auto tic=std::chrono::steady_clock::now();Result result;double rho=.01;
    Vec a(S,0),w(S,0),oldw(S),v(S,0),z(2*S,0),oldz(2*S),u(2*S,0),g(2*S),temp(2*S),ad(S),rhs(S),ri(S),ktx(S),ky(S);
    Vec spectral_scale(S);
    double cached_rho=-1.;
    RInverse rinv;const double dlim=1e-7*std::max(1.,norm(meas.h));
    PS_STOP(profile_setup);
    for(int it=1;it<=limit;++it){
        PS_SCOPE(profile_iteration, Iteration);
        PS_SCOPE(profile_phase, RHS);
        const bool check_residual=(it%100==0||it==limit);
        for(int i=0;i<2*S;++i)temp[i]=z[i]-u[i];
        adj(temp,ad);
        for(int i=0;i<S;++i)rhs[i]=meas.h[i]+rho*(ad[i]+w[i]-v[i]);
        PS_NEXT(profile_phase, DCT);
        rinv.run(rhs,m.rspec,ri);
        PS_NEXT(profile_phase, KT);
        backend.multiply(true,rhs,ktx);
        PS_NEXT(profile_phase, Scale);
        // rho is the only changing input to these factors within a solve.
        if(cached_rho!=rho){
            for(int i=0;i<N;++i)for(int c=0;c<3;++c){
                const double cc2=meas.cc[c]*meas.cc[c];
                spectral_scale[i*3+c]=cc2/(rho+m.ev[i]*cc2);
            }
            cached_rho=rho;
        }
        for(int i=0;i<S;++i)ktx[i]*=spectral_scale[i];
        PS_NEXT(profile_phase, K);
        backend.multiply(false,ktx,ky);
        PS_NEXT(profile_phase, VectorUpdate);
        for(int i=0;i<S;++i)a[i]=(ri[i]-ky[i])/rho;
        grad(a,g);
        // Residuals compare this update with the immediately preceding state.
        if(check_residual){oldz=z;oldw=w;}
        for(int i=0;i<S;++i){
            double t0=g[i]+u[i],t1=g[S+i]+u[S+i];
            double scale=std::max(0.,1-.001/rho/std::max(std::sqrt(t0*t0+t1*t1),1e-30));
            z[i]=t0*scale;z[S+i]=t1*scale;w[i]=std::max(0.,a[i]+v[i]);
            u[i]+=g[i]-z[i];u[S+i]+=g[S+i]-z[S+i];v[i]+=a[i]-w[i];}
        if(check_residual){
            PS_NEXT(profile_phase, Residual);
            finite(w);finite(u);finite(v);double pr=0;
            for(int i=0;i<2*S;++i){double d=g[i]-z[i];pr+=d*d;temp[i]=z[i]-oldz[i];}
            for(int i=0;i<S;++i){double d=a[i]-w[i];pr+=d*d;}
            adj(temp,ad);for(int i=0;i<S;++i)ad[i]+=w[i]-oldw[i];
            result.primal=std::sqrt(pr);result.dual=rho*norm(ad);result.plim=1e-7*std::max(1.,norm(w));result.dlim=dlim;
            result.passed=result.primal<result.plim&&result.dual<dlim;
            result.trace.push_back({double(it),result.primal,result.dual,rho,result.plim,dlim});
            result.iterations=it;result.rho=rho;if(result.passed)break;
            if(it<limit){double next=rho;
                if(result.primal/result.plim>10*result.dual/dlim)next=std::min(.1,rho*2);
                else if(result.dual/dlim>10*result.primal/result.plim)next=std::max(1e-5,rho/2);
                if(next!=rho){for(double& x:u)x*=rho/next;for(double& x:v)x*=rho/next;rho=next;}}
        }
    }
    result.f=w;result.seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-tic).count();return result;
}
inline void save_result(const std::string& prefix,const Model& m,const Measurement& meas,const Result& r,const std::string& backend){
    require(!std::ifstream(prefix+".pfo")&&!std::ifstream(prefix+".json"),"Refusing to overwrite result");
    Vec payload=meas.h;payload.insert(payload.end(),r.f.begin(),r.f.end());for(double x:r.f)payload.push_back(x*meas.multiplier);finite(payload);
    unsigned char hdr[64]={};std::memcpy(hdr,"PSFULL1\0",8);
    auto put=[&](int off,auto val){std::memcpy(hdr+off,&val,sizeof(val));};
    put(8,uint32_t(1));put(12,uint32_t(4));put(16,uint32_t(H));put(20,uint32_t(W));put(24,uint64_t(payload.size()*8));
    put(32,crc32(reinterpret_cast<const unsigned char*>(payload.data()),payload.size()*8));std::copy(m.id.begin(),m.id.end(),hdr+40);
    std::ofstream f(prefix+".pfo",std::ios::binary);f.write(reinterpret_cast<char*>(hdr),64);f.write(reinterpret_cast<const char*>(payload.data()),payload.size()*8);require(bool(f),"Output write failed");f.close();
    std::ofstream j(prefix+".json");j<<std::setprecision(17)<<"{\n\"backend\":\""<<backend<<"\",\n\"converged\":"<<(r.passed?"true":"false")
      <<",\n\"iterations\":"<<r.iterations<<",\n\"rho\":"<<r.rho<<",\n\"primal\":"<<r.primal<<",\n\"dual\":"<<r.dual
      <<",\n\"primal_limit\":"<<r.plim<<",\n\"dual_limit\":"<<r.dlim<<",\n\"seconds_solver\":"<<r.seconds
      <<",\n\"physical_multiplier\":"<<meas.multiplier<<",\n\"high_confidence\":false,\n\"confidence_probability\":null,\n\"status\":\""
      <<(r.passed?"uncalibrated":"numerical_failure")<<"\",\n\"input_quality\":{\"flags\":[";
    for(size_t i=0;i<meas.flags.size();++i){if(i)j<<',';j<<'\"'<<meas.flags[i]<<'\"';}
    j<<"],\"gradient_relative_norm\":"<<meas.gradient_relative_norm<<",\"gradient_rms\":"<<meas.gradient_rms
     <<",\"spatial_noise_proxy_per_channel\":["<<meas.noise[0]<<','<<meas.noise[1]<<','<<meas.noise[2]
     <<"],\"gradient_to_noise_proxy\":"<<meas.noise_proxy<<",\"isolated_outlier_ratio\":"<<meas.outlier_ratio
     <<",\"saturation_fraction\":null,\"note\":\"Engineering input diagnostics; sensor full scale and confidence remain uncalibrated\"},\n\"trace\":[";
    for(size_t i=0;i<r.trace.size();++i){if(i)j<<',';j<<'[';for(int c=0;c<6;++c){if(c)j<<',';j<<r.trace[i][c];}j<<']';}j<<"]\n}\n";require(bool(j),"JSON write failed");
    // Nearest-neighbour display only. PFO preserves unscaled FP64 arrays.
    std::ofstream p(prefix+".ppm",std::ios::binary);p<<"P6\n"<<W<<' '<<H<<"\n255\n";
    const double mx=std::max(*std::max_element(r.f.begin(),r.f.end()),1e-30);
    for(double x:r.f){unsigned char b=static_cast<unsigned char>(std::round(std::min(1.,std::max(0.,x/mx))*255));p.write(reinterpret_cast<const char*>(&b),1);}require(bool(p),"Preview write failed");
}
}
