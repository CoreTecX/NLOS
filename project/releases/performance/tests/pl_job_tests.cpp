#include "../src/full_runtime.hpp"
#include "../src/pl_job.hpp"
#include <functional>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <thread>
#endif
using namespace ps;
struct FakeIO:JobIO {
    uint64_t clock=0,delay=0,ready=0;unsigned n=0,status=ps_layout::OK;
    bool running=false,stopped=false,stale=false,never=false,nonfinite=false;
    int copies=0,starts=0,reads=0,pauses=0;Vec input,output=Vec(S,7.);
    uint64_t now_ns()override{return clock;}
    void pause()override{clock+=20000;++pauses;}
    uint32_t read_reg(unsigned off)override{
        if(off==ps_layout::REG_STATUS)return n?status:ps_layout::BAD_N;
        ++reads;
        if(running){if(!never&&clock>=ready){running=false;return 6;}return 0;}
        if(stale){stale=false;return 6;}return 4;
    }
    void write_reg(unsigned off,uint32_t value)override{
        if(off==ps_layout::REG_N)n=value;
        if(off==0&&value==1){running=true;ready=clock+delay;++starts;}
    }
    void copy_input(bool,const Vec& x)override{input=x;++copies;}
    void copy_output(Vec& y)override{y=output;if(nonfinite)y[0]=std::numeric_limits<double>::infinity();}
    void stop_hardware()noexcept override{stopped=true;running=false;}
};
int checks=0;
void check(bool x){require(x,"PL protocol assertion");++checks;}
void rejects(const std::function<void()>& f){try{f();}catch(const std::exception&){++checks;return;}throw std::runtime_error("Expected rejection");}
void other_thread(const std::function<void()>& callback){
#ifdef _WIN32
    auto invoke=[](LPVOID p)->DWORD{(*static_cast<const std::function<void()>*>(p))();return 0;};
    HANDLE thread=CreateThread(nullptr,0,invoke,const_cast<std::function<void()>*>(&callback),0,nullptr);
    require(thread!=nullptr,"Thread creation failed");WaitForSingleObject(thread,INFINITE);CloseHandle(thread);
#else
    std::thread thread(callback);thread.join();
#endif
}
int main(){try{
    Vec x(S,3.),y(S,0.);
    {FakeIO io;PLJob job(io);job.probe();check(io.starts==1&&job.state()==PLJob::State::Idle);io.stale=true;
     job.begin(true,x);x[0]=4.;check(io.input[0]==3.);rejects([&]{job.begin(false,x);});check(io.copies==1);
     bool rejected=false;other_thread([&]{try{job.finish(y);}catch(...){rejected=true;}});check(rejected);
     job.finish(y);check(y[0]==7.&&io.starts==2&&!io.stopped);rejects([&]{job.finish(y);});
     job.multiply(false,x,y);check(io.starts==3);}
    {FakeIO io;io.delay=100000;PLJob job(io);job.begin(true,x);io.clock=40000;job.finish(y);check(io.pauses==3);}
    {FakeIO io;PLJob job(io);job.begin(true,x);io.clock=4999999999ull;job.finish(y);check(!io.stopped);}
    {FakeIO io;PLJob job(io);job.begin(true,x);io.clock=5000000000ull;rejects([&]{job.finish(y);});check(io.stopped);rejects([&]{job.begin(false,x);});check(io.copies==1);}
    {FakeIO io;io.never=true;PLJob job(io);job.begin(true,x);io.clock=4999980000ull;rejects([&]{job.finish(y);});check(io.clock==5000000000ull&&io.stopped);}
    {FakeIO io;io.status=ps_layout::BAD_LAYOUT;PLJob job(io);job.begin(true,x);rejects([&]{job.finish(y);});check(io.stopped);rejects([&]{job.probe();});}
    {FakeIO io;io.nonfinite=true;PLJob job(io);job.begin(false,x);rejects([&]{job.finish(y);});check(io.stopped);}
    {FakeIO io;io.running=true;io.never=true;PLJob job(io);rejects([&]{job.begin(false,x);});check(io.copies==0&&io.stopped);}
    {FakeIO io;PLJob job(io);Vec bad(1);rejects([&]{job.begin(false,bad);});check(io.copies==0&&!io.stopped);x[0]=NAN;rejects([&]{job.begin(false,x);});x[0]=INFINITY;rejects([&]{job.begin(false,x);});x[0]=1.;job.begin(false,x);rejects([&]{job.finish(bad);});job.finish(y);check(!io.stopped);}
    {FakeIO io;PLJob job(io);job.begin(true,x);job.stop();check(io.stopped);rejects([&]{job.finish(y);});
     // Reload constructs fresh protocol and refills matrices/input at same address.
     FakeIO reloaded;PLJob next(reloaded);next.probe();x[0]=123.;next.multiply(false,x,y);check(reloaded.input[0]==123.&&!reloaded.stopped);}
    std::cout<<"{\"passed\":true,\"checks\":"<<checks<<",\"hardware_access\":false}\n";
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
