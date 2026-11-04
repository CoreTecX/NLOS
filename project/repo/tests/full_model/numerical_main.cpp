#define PS_EXPERIMENTAL_SOLVER
#include "../src/full_runtime.hpp"
#include "experimental/mixed_dct.hpp"
#include "experimental/fma_dct.hpp"
#include "../src/pl_job.hpp"
// Emulated IO executes the immutable reference kernel on finish's first poll.
// This exercises the production PLJob protocol and numerical schedule, not DDR.
class EmulatedPL:public ps::Backend,private ps::JobIO {
    const ps::Model& model;ps::PLJob job{*this};ps::Vec input,output=ps::Vec(ps::S);
    bool trans=false,running=false;uint64_t time=0;unsigned status=ps_layout::OK;
    uint64_t now_ns()override{return time;}
    void pause()override{time+=20000;}
    uint32_t read_reg(unsigned off)override{
        if(off==ps_layout::REG_STATUS)return status;
        if(running){running=false;status=ps::reference_gemv((trans?model.kt:model.k).data(),input.data(),output.data(),ps::N)==0x46500100u?ps_layout::OK:ps_layout::BAD_N;return 6;}return 4;
    }
    void write_reg(unsigned off,uint32_t value)override{if(off==0&&value==1)running=true;}
    void copy_input(bool t,const ps::Vec& x)override{input=x;trans=t;}
    void copy_output(ps::Vec& y)override{y=output;}
    void stop_hardware()noexcept override{running=false;}
public:
    explicit EmulatedPL(const ps::Model& m):model(m){}
    bool asynchronous()const override{return true;}
    void begin(bool t,const ps::Vec& x)override{job.begin(t,x);}
    void finish(ps::Vec& y)override{job.finish(y);}
    void cancel()noexcept override{job.stop();}
    void multiply(bool t,const ps::Vec& x,ps::Vec& y)override{job.multiply(t,x,y);}
};
int main(int argc,char** argv){try{
    ps::require(argc==5,"MODEL MEAS OUT POLICY");const std::string policy=argv[4];
    ps::Model model(argv[1]);ps::Measurement meas(argv[2],model);model.release_measurement_basis();
    ps::Host host(model);ps::SolverOptions options;ps::SolverState state;options.state=&state;
    ps::Result result;
    if(policy=="rho005")options.initial_rho=.005;
    else if(policy=="rho02")options.initial_rho=.02;
    else if(policy=="relax15")options.relaxation=1.5;
    else if(policy=="check50")options.check_every=50;
    else if(policy=="check200")options.check_every=200;
    else ps::require(policy=="strict"||policy=="mixed"||policy=="fma"||policy=="async"||policy=="sync","Unknown candidate");
    if(policy=="mixed")result=ps::solve<ps::MixedRInverse<29,36>>(model,meas,host,20000,options);
    else if(policy=="fma")result=ps::solve<ps::FmaRInverse<29,36>>(model,meas,host,20000,options);
    else if(policy=="async"||policy=="sync"){EmulatedPL emulated(model);options.overlap=policy=="async";result=ps::solve(model,meas,emulated,20000,options);}
    else result=ps::solve(model,meas,host,20000,options);
    ps::save_result(argv[3],model,meas,result,"cpu");ps::save_state(argv[3],model,meas,result,state,options.relaxation);
    std::cout<<policy<<" iterations="<<result.iterations<<" passed="<<result.passed<<'\n';return result.passed?0:3;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
