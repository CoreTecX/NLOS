#pragma once
#include <array>
#include <complex>
#include <cmath>
#include <vector>
namespace ps {
// Bounded, dependency-free experiment: direct DCT29 and radix2/3 FFT72 DCT36.
// Both forward/inverse are orthonormal DCT-II/III with the original boundaries.
template<int L> class SmallFFT {
    static constexpr int R=L%2==0?2:3, Q=L/R;
    std::array<std::complex<double>,L> roots;
    SmallFFT<Q> child;
public:
    SmallFFT(){for(int k=0;k<L;++k)roots[k]=std::polar(1.,-2.*std::acos(-1.)*k/L);}
    void run(const std::complex<double>* in,int stride,std::complex<double>* out,bool inverse)const{
        std::array<std::complex<double>,L> scratch;
        for(int r=0;r<R;++r)child.run(in+r*stride,stride*R,scratch.data()+r*Q,inverse);
        for(int k=0;k<L;++k){std::complex<double> sum=0.;for(int r=0;r<R;++r){auto w=roots[(r*k)%L];sum+=scratch[r*Q+k%Q]*(inverse?std::conj(w):w);}out[k]=sum;}
    }
};
template<> class SmallFFT<1>{public:void run(const std::complex<double>* in,int,std::complex<double>* out,bool)const{out[0]=in[0];}};
template<int H,int W> class MixedRInverse {
    static_assert(H==29&&W==36,"Bounded candidate for current model only");
    double ch[H][H],cht[H][H],scale[W];
    std::complex<double> phase[W];SmallFFT<2*W> fft;
    std::vector<double> temp,buffer;
    void height(const std::vector<double>& in,std::vector<double>& out,bool inverse){
        for(int k=0;k<H;++k)for(int q=0;q<W*3;++q){double s=0;for(int r=0;r<H;++r)s+=(inverse?cht[k][r]:ch[k][r])*in[r*W*3+q];out[k*W*3+q]=s;}
    }
    void width(const std::vector<double>& in,std::vector<double>& out,bool inverse){
        for(int r=0;r<H;++r)for(int c=0;c<3;++c){
            std::array<std::complex<double>,2*W> a{},b{};
            if(!inverse){
                for(int j=0;j<W;++j)a[j]=a[2*W-1-j]=in[(r*W+j)*3+c];
                fft.run(a.data(),1,b.data(),false);
                for(int k=0;k<W;++k)out[(r*W+k)*3+c]=(b[k]*std::conj(phase[k])).real()*.5*scale[k];
            }else{
                for(int k=0;k<W;++k){a[k]=(2.*in[(r*W+k)*3+c]/scale[k])*phase[k];if(k)a[2*W-k]=std::conj(a[k]);}
                fft.run(a.data(),1,b.data(),true);
                for(int j=0;j<W;++j)out[(r*W+j)*3+c]=b[j].real()/(2*W);
            }
        }
    }
public:
    MixedRInverse():temp(H*W*3),buffer(H*W*3){
        for(int k=0;k<H;++k)for(int r=0;r<H;++r)cht[r][k]=ch[k][r]=std::sqrt((k?2.:1.)/H)*std::cos(std::acos(-1.)*(r+.5)*k/H);
        for(int k=0;k<W;++k){scale[k]=std::sqrt((k?2.:1.)/W);phase[k]=std::polar(1.,std::acos(-1.)*k/(2*W));}
    }
    void run(const std::vector<double>& in,const std::vector<double>& spec,std::vector<double>& out){
        height(in,temp,false);width(temp,buffer,false);
        for(int i=0;i<H*W*3;++i)buffer[i]/=spec[i/3];
        height(buffer,temp,true);width(temp,out,true);
    }
};
}
