#pragma once
#include "gemv_contract.hpp"
#include <cmath>
#include <cstdint>
#include <atomic>
#include <stdexcept>
#include <vector>

namespace ps {
// The same protocol runs with MMIO on A53 and deterministic IO in fault tests.
// IO owns reserved, uncached DDR. A barrier publishes input before ap_start and
// acquires output after ap_done. The caller's input is never retained by PL.
struct JobIO {
    virtual uint64_t now_ns()=0;
    virtual void pause()=0;
    virtual uint32_t read_reg(unsigned)=0;
    virtual void write_reg(unsigned,uint32_t)=0;
    virtual void copy_input(bool,const std::vector<double>&)=0;
    virtual void copy_output(std::vector<double>&)=0;
    virtual void stop_hardware() noexcept=0;
    virtual ~JobIO()=default;
};
class PLJob {
public:
    enum class State { Idle, Running, Faulted };
private:
    JobIO& io;
    std::atomic_flag gate=ATOMIC_FLAG_INIT;
    struct Guard {
        std::atomic_flag& gate;
        explicit Guard(std::atomic_flag& flag):gate(flag){if(gate.test_and_set(std::memory_order_acquire))throw std::runtime_error("Concurrent PL API use rejected");}
        ~Guard(){gate.clear(std::memory_order_release);}
        Guard(const Guard&)=delete;
    };
    State state_=State::Idle;
    uint64_t submitted=0;
    unsigned expected=ps_layout::OK;
    bool probe_=false;
    const void* owner=nullptr;
    static const void* thread_token(){static thread_local char token;return &token;}
    static constexpr uint64_t timeout_ns=5000000000ull;
    [[noreturn]] void fail(const char* message){
        state_=State::Faulted;io.stop_hardware();throw std::runtime_error(message);
    }
    void start(unsigned n,unsigned status){
        // ap_done is clear-on-read. Drain an old completion only while idle;
        // never read AP_CTRL elsewhere during a live operation.
        if(!(io.read_reg(0)&4))fail("PL not idle: reset/reload required");
        io.write_reg(ps_layout::REG_LAYOUT,ps_layout::VERSION);
        io.write_reg(ps_layout::REG_N,n);expected=status;
        submitted=io.now_ns();state_=State::Running;owner=thread_token();
        io.write_reg(0,1);
    }
    void await_done(){
        for(;;){
            // Conservative: a completion first observed beyond the deadline
            // is a timeout even if hardware may have finished during CPU work.
            if(io.now_ns()-submitted>=timeout_ns)fail("PL timeout from submit: reset/reload required");
            if(io.read_reg(0)&2)break;
            io.pause();
        }
        if(io.read_reg(ps_layout::REG_STATUS)!=expected)fail("PL status error: reset/reload required");
    }
    void usable(){
        if(state_==State::Faulted)throw std::runtime_error("PL is faulted: create a new backend after reset/reload");
    }
public:
    explicit PLJob(JobIO& device):io(device){}
    PLJob(const PLJob&)=delete;PLJob& operator=(const PLJob&)=delete;
    void probe(){
        Guard lock(gate);usable();
        if(state_!=State::Idle)throw std::runtime_error("PL already running");
        try{probe_=true;start(0,ps_layout::BAD_N);await_done();state_=State::Idle;probe_=false;}
        catch(...){state_=State::Faulted;io.stop_hardware();throw;}
    }
    void begin(bool transpose,const std::vector<double>& x){
        Guard lock(gate);usable();
        if(state_!=State::Idle)throw std::runtime_error("PL already running; finish before resubmission");
        if(x.size()!=3*ps_layout::MAX_N)throw std::invalid_argument("PL input size");
        for(double value:x)if(!std::isfinite(value))throw std::invalid_argument("Nonfinite PL input");
        // Reject a live/stale device before writing its buffers.
        if(!(io.read_reg(0)&4))fail("PL not idle before input copy: reset/reload required");
        try{io.copy_input(transpose,x);start(ps_layout::MAX_N,ps_layout::OK);}
        catch(...){state_=State::Faulted;io.stop_hardware();throw;}
    }
    void finish(std::vector<double>& y){
        Guard lock(gate);usable();
        if(state_!=State::Running||probe_)throw std::runtime_error("No pending PL result");
        if(owner!=thread_token())throw std::runtime_error("Only the submitting thread may finish PL");
        if(y.size()!=3*ps_layout::MAX_N)throw std::invalid_argument("PL output size; pending job still owned");
        try{
            await_done();io.copy_output(y);
            for(double value:y)if(!std::isfinite(value))fail("Nonfinite PL output: reset/reload required");
            state_=State::Idle;
        }catch(...){state_=State::Faulted;io.stop_hardware();throw;}
    }
    void multiply(bool transpose,const std::vector<double>& x,std::vector<double>& y){begin(transpose,x);finish(y);}
    // Called on exceptional solver exit/destruction. No CPU fallback or buffer
    // reuse is allowed until external reset/reload and a fresh backend.
    void stop() noexcept {state_=State::Faulted;io.stop_hardware();}
    State state()const{return state_;} // Owner-thread diagnostic only.
};
}
