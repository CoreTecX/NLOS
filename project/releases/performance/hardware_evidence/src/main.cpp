#include "full_runtime.hpp"
#include "pl_backend.hpp"
#ifdef PS_TEST_PACKED_HOST
#include "../tests/packed_host.hpp"
#endif
int main(int argc,char** argv){
    try{
        ps::require(argc==5||argc==6,"Usage: ps_full MODEL.pfm MEASUREMENT.pfy OUTPUT_PREFIX cpu|pl [LIMIT]");
        PS_PROFILE_INIT(argv[3]);
        PS_SCOPE(profile_main, Main);
        PS_SCOPE(profile_input, Arguments);
        const std::string mode=argv[4];ps::require(mode=="cpu"||mode=="pl","Backend must be cpu or pl");
        int limit=20000;if(argc==6){size_t end=0;limit=std::stoi(argv[5],&end);ps::require(end==std::strlen(argv[5])&&limit>0&&limit<=20000,"Invalid iteration limit");}
        ps::require(!std::ifstream(std::string(argv[3])+".pfo")&&!std::ifstream(std::string(argv[3])+".json"),"Refusing to overwrite result");
        PS_NEXT(profile_input, ModelLoad);
        ps::Model m(argv[1]);
        PS_NEXT(profile_input, MeasurementLoad);
        ps::Measurement meas(argv[2],m);m.release_measurement_basis();std::cout<<"INPUT_VALIDATED"<<std::endl;
        PS_STOP(profile_input);
        ps::Result r;
        if(mode=="cpu"){
            PS_SCOPE(profile_backend, BackendPrepare);
#ifdef PS_TEST_PACKED_HOST
            ps::PackedHost host(m);m.release_cpu_matrices();
#else
            ps::Host host(m);
#endif
            PS_STOP(profile_backend);
            r=ps::solve(m,meas,host,limit);}
        else {PS_SCOPE(profile_backend, BackendPrepare);ps::PL pl(m);m.release_cpu_matrices();PS_STOP(profile_backend);r=ps::solve(m,meas,pl,limit);}
        {PS_SCOPE(profile_output, Output);ps::save_result(argv[3],m,meas,r,mode);}
        std::cout<<"CONVERGED="<<r.passed<<" ITERATIONS="<<r.iterations<<" SECONDS="<<r.seconds<<std::endl;
        PS_STOP(profile_main);PS_PROFILE_WRITE(argv[3]);
        return r.passed?0:3;
    }catch(const std::exception& e){std::cerr<<"REJECTED: "<<e.what()<<std::endl;return 2;}
}
