#define PS_ENABLE_PHASE_PROFILE
#include "../src/phase_profile.hpp"
#include <iostream>
#include <stdexcept>
#include <cassert>
int main(){
    using namespace ps_profile;
    enabled=false;{Scope root(Main);Scope child(DCT);}assert(stats[Main].calls==0);
    enabled=true;
    {Scope root(Main);{Scope child(DCT);child.next(KT);}
        try{Scope failing(Output);throw std::runtime_error("test");}catch(const std::runtime_error&){}
        assert(current==&root);
    }
    assert(current==nullptr&&stats[Main].calls==1&&stats[DCT].calls==1&&stats[KT].calls==1&&stats[Output].calls==1);
    assert(stats[Main].inclusive==stats[Main].exclusive+stats[DCT].inclusive+stats[KT].inclusive+stats[Output].inclusive);
    volatile uint64_t sink=0;
    std::cout<<"{\"passed\":true,\"scope_pairs\":1000000,\"scope_cost_ns\":[";
    for(int repeat=0;repeat<7;++repeat){
        auto tic=tick();for(int i=0;i<1000000;++i){sink+=unsigned(i);}
        const double baseline=double(tick()-tic);
        tic=tick();for(int i=0;i<1000000;++i){Scope s(DCT);sink+=unsigned(i);}
        const double observed=double(tick()-tic);
        if(repeat)std::cout<<',';
        std::cout<<(observed-baseline)/1000000.;
    }
    int64_t minimum=1000000000;auto previous=tick();
    for(int i=0;i<100000;++i){auto now=tick();if(now>previous&&now-previous<minimum)minimum=now-previous;previous=now;}
    assert(minimum<1000000);
    std::cout<<"],\"minimum_observed_clock_step_ns\":"<<minimum<<",\"sink\":"<<sink<<"}\n";
}
