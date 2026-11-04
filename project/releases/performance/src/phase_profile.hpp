#pragma once
// Optional observation only: no numerical state, hardware ABI or scheduling change.
// The normal build expands every macro to nothing and reads no profile environment.
#ifdef PS_ENABLE_PHASE_PROFILE
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif
namespace ps_profile {
enum Phase { Main, Arguments, ModelLoad, MeasurementLoad, InputRead, InputCRC,
    InputDiagnostics, HConstruct, BackendPrepare, MatrixPack, Solver, SolverSetup,
    Iteration, RHS, DCT, KT, Scale, K, VectorUpdate, Residual,
    CopyIn, Submit, Wait, CopyOut, Output, Count };
inline constexpr const char* names[Count]={"main","arguments","model_load","measurement_load",
    "input_read","input_crc","input_diagnostics","h_construct","backend_prepare","matrix_pack",
    "solver","solver_setup","iteration","rhs","dct","kt_gemv","spectral_scale","k_gemv",
    "vector_update","residual_check","driver_copy_in","driver_submit","driver_wait",
    "driver_copy_out","output"};
struct Stat { uint64_t calls=0; int64_t inclusive=0,exclusive=0; };
inline std::array<Stat,Count> stats{};
inline bool enabled=false;
using Clock=std::chrono::steady_clock;
inline int64_t tick(){
#ifdef _WIN32
    static const int64_t frequency=[](){LARGE_INTEGER value;QueryPerformanceFrequency(&value);return value.QuadPart;}();
    LARGE_INTEGER value;QueryPerformanceCounter(&value);
    return (value.QuadPart/frequency)*1000000000ll+(value.QuadPart%frequency)*1000000000ll/frequency;
#else
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
#endif
}
struct Scope;
inline Scope* current=nullptr; // Runtime is single threaded. Future threaded kernels need per-thread collectors.
struct Scope {
    Phase phase; bool active=false; int64_t started=0; int64_t children=0; Scope* parent=nullptr;
    explicit Scope(Phase p):phase(p){start();}
    Scope(const Scope&)=delete; Scope& operator=(const Scope&)=delete;
    void start(){if(!enabled)return;active=true;children=0;parent=current;current=this;started=tick();}
    void stop(){if(!active)return;const auto elapsed=tick()-started;
        auto& s=stats[phase];++s.calls;s.inclusive+=elapsed;s.exclusive+=elapsed-children;
        current=parent;if(parent)parent->children+=elapsed;active=false;}
    void next(Phase p){stop();phase=p;start();}
    ~Scope(){stop();}
};
inline void init(const std::string& prefix){
    const char* value=std::getenv("PS_PROFILE");enabled=value&&std::string(value)=="1";
    if(enabled&&std::ifstream(prefix+".profile.json"))throw std::runtime_error("Refusing to overwrite profile");
}
inline void write(const std::string& prefix){
    if(!enabled)return;
    std::ofstream f(prefix+".profile.json");
    f<<std::setprecision(17)<<"{\n\"schema\":\"ps-phase-profile-v1\",\"clock\":\"QPC_on_Windows_steady_clock_elsewhere\","
        "\"unit\":\"seconds\",\"physical_board_measurement\":null,"
        "\"note\":\"Execution platform is recorded by the runner; nested inclusive stages must not be summed.\",\"phases\":{\n";
    for(int i=0;i<Count;++i){if(i)f<<",\n";const auto& s=stats[i];
        f<<'\"'<<names[i]<<"\":{\"calls\":"<<s.calls<<",\"inclusive_seconds\":"<<double(s.inclusive)*1e-9
            <<",\"exclusive_seconds\":"<<double(s.exclusive)*1e-9<<'}';}
    f<<"\n}}\n";if(!f)throw std::runtime_error("Profile write failed");
}
}
#define PS_SCOPE(variable,phase) ps_profile::Scope variable(ps_profile::phase)
#define PS_NEXT(variable,phase) variable.next(ps_profile::phase)
#define PS_STOP(variable) variable.stop()
#define PS_PROFILE_INIT(prefix) ps_profile::init(prefix)
#define PS_PROFILE_WRITE(prefix) ps_profile::write(prefix)
#else
#define PS_SCOPE(variable,phase) ((void)0)
#define PS_NEXT(variable,phase) ((void)0)
#define PS_STOP(variable) ((void)0)
#define PS_PROFILE_INIT(prefix) ((void)0)
#define PS_PROFILE_WRITE(prefix) ((void)0)
#endif
