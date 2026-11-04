#include "../src/full_runtime.hpp"
#include "../src/row_pair.hpp"
void hexfile(const std::string& path,const ps::Vec& a,size_t count){
    std::ofstream f(path);f<<std::hex<<std::setfill('0');
    for(size_t i=0;i<count;++i){uint64_t v;std::memcpy(&v,&a[i],8);f<<std::setw(16)<<v<<'\n';}
    ps::require(bool(f),"RTL vector write failed");
}
void matrixfile(const std::string& path,const ps::Vec& a,unsigned n){
    auto p=ps_layout::pack(a,n);std::ofstream f(path);f<<std::hex<<std::setfill('0');
    for(const auto& word:p){uint64_t lo,hi;std::memcpy(&lo,&word.lo,8);std::memcpy(&hi,&word.hi,8);
        f<<std::setw(16)<<hi<<std::setw(16)<<lo<<'\n';}
    ps::require(bool(f),"RTL matrix write failed");
}
int main(int argc,char** argv){
    try{ps::require(argc==4,"Usage: rtl_vectors model measurement output_directory");
        ps::Model m(argv[1]);ps::Measurement y(argv[2],m);const std::string dir=std::string(argv[3])+"/";
        matrixfile(dir+"matrix_kt.hex",m.kt,ps::N);matrixfile(dir+"matrix_k.hex",m.k,ps::N);
        hexfile(dir+"input.hex",y.h,y.h.size());ps::Vec result(ps::S),temp(ps::S);
        for(unsigned n:{1u,2u,17u,64u,257u,1043u,1044u}){
            ps::Vec a(size_t(n)*n);
            for(unsigned r=0;r<n;++r)for(unsigned j=0;j<n;++j)a[size_t(r)*n+j]=m.kt[size_t(r)*ps::N+j];
            matrixfile(dir+"matrix_"+std::to_string(n)+".hex",a,n);
            ps::require(ps::reference_gemv(a.data(),y.h.data(),result.data(),n)==0x46500100u,"Reference failure");
            hexfile(dir+"expected_"+std::to_string(n)+".hex",result,n*3);
        }
        for(int i=0;i<ps::N;++i)for(int c=0;c<3;++c){double cc=y.cc[c]*y.cc[c];temp[3*i+c]=result[3*i+c]*cc/(.01+m.ev[i]*cc);}
        hexfile(dir+"input_forward.hex",temp,temp.size());
        ps::require(ps::reference_gemv(m.k.data(),temp.data(),result.data(),ps::N)==0x46500100u,"Reference failure");
        hexfile(dir+"expected_forward.hex",result,result.size());
        std::cout<<"D11_RGB_VECTORS_FROM_FROZEN_ROW_MAJOR_REFERENCE_PASS"<<std::endl;
    }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
}
