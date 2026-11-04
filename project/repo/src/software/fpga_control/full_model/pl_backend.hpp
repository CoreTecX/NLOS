#pragma once
#include "full_runtime.hpp"
#include "row_pair.hpp"
#include "pl_job.hpp"
#include <memory>
#if defined(__linux__) && defined(__aarch64__)
#include <fcntl.h>
#include <unistd.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <dirent.h>
namespace ps {
// Register base is intentionally different from the old rank64 pipeline.
// DDR must be a live device-tree no-map reservation; never guess a free address.
class PL:public Backend,private JobIO {
    PLJob job{*this};
    volatile uint32_t* gpio=nullptr;
    int fd=-1,lock=-1;void* mapping=MAP_FAILED;volatile uint32_t* regs=nullptr;
    uint64_t base=0;static constexpr size_t BYTES=32*1024*1024;
    static constexpr size_t KO=0,KTO=9*1024*1024,XO=18*1024*1024,YO=19*1024*1024;
    static uint64_t be(const std::vector<unsigned char>& a,size_t p,int cells){uint64_t v=0;for(int i=0;i<cells*4;++i)v=v*256+a.at(p+i);return v;}
    static std::vector<unsigned char> read(const std::string& p){std::ifstream f(p,std::ios::binary);return {std::istreambuf_iterator<char>(f),{}};}
    static void barrier(){asm volatile("dsb sy":::"memory");}
    void wr(unsigned off,uint32_t v){regs[off/4]=v;barrier();}
    uint32_t rd(unsigned off){uint32_t v=regs[off/4];barrier();return v;}
    void pointer(unsigned off,uint64_t addr){wr(off,uint32_t(addr));wr(off+4,uint32_t(addr>>32));}
    uint64_t now_ns()override{return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());}
    void pause()override{usleep(20);}
    uint32_t read_reg(unsigned off)override{return rd(off);}
    void write_reg(unsigned off,uint32_t value)override{wr(off,value);}
    void copy_input(bool transpose,const Vec& x)override{
        {PS_SCOPE(profile_copy, CopyIn);std::memcpy(static_cast<char*>(mapping)+XO,x.data(),S*8);barrier();}
        PS_SCOPE(profile_submit, Submit);pointer(0x10,base+(transpose?KTO:KO));
    }
    void copy_output(Vec& y)override{
        PS_SCOPE(profile_copy, CopyOut);barrier();std::memcpy(y.data(),static_cast<char*>(mapping)+YO,S*8);
    }
    void stop_hardware()noexcept override{
        // Same EMIO PL-reset line as verified run_target.py; hold reset low.
        if(gpio){gpio[0x54/4]=gpio[0x54/4]&~0x80000000u;barrier();}
    }
    void cleanup()noexcept{
        if(job.state()!=PLJob::State::Idle)job.stop();
        if(gpio){munmap(const_cast<uint32_t*>(gpio),4096);gpio=nullptr;}
        if(regs){munmap(const_cast<uint32_t*>(regs),4096);regs=nullptr;}
        if(mapping!=MAP_FAILED){munmap(mapping,BYTES);mapping=MAP_FAILED;}
        if(fd>=0){close(fd);fd=-1;}if(lock>=0){close(lock);lock=-1;}
    }

public:
    explicit PL(const Model& m){try{
        m.require_cpu_matrices();
        require(std::getenv("PS_FULL_LOADED")&&std::string(std::getenv("PS_FULL_LOADED"))=="20261016-rp2-p56-v1", "Use verified run_target.py --backend pl launcher");
        lock=open("/var/lock/psight-full.lock",O_CREAT|O_RDWR,0600);require(lock>=0&&flock(lock,LOCK_EX|LOCK_NB)==0,"Another full-model process holds DDR");
        const std::string root="/sys/firmware/devicetree/base/reserved-memory";
        auto ac=read(root+"/#address-cells"),sc=read(root+"/#size-cells");
        require(ac.size()==4&&sc.size()==4,"Cannot validate reserved DDR cells");int na=int(be(ac,0,1)),ns=int(be(sc,0,1));
        require((na==1||na==2)&&(ns==1||ns==2),"Unsupported reserved-memory cells");
        std::unique_ptr<DIR,int(*)(DIR*)> directory(opendir(root.c_str()),closedir);
        DIR* d=directory.get();require(d,"Reserved DDR unavailable");
        for(dirent* e=readdir(d);e;e=readdir(d)){
            std::string node=root+"/"+e->d_name;
            if(std::strncmp(e->d_name,"penumbrasight",13)!=0 && std::strncmp(e->d_name,"psight",6)!=0)continue;
            auto reg=read(node+"/reg");
            if(access((node+"/no-map").c_str(),F_OK)!=0||reg.size()!=size_t((na+ns)*4))continue;
            uint64_t b=be(reg,0,na),sz=be(reg,na*4,ns);
            if(b>=0x10000000&&b%4096==0&&b<0x80000000ull&&sz>=BYTES&&sz<=0x80000000ull-b){require(base==0,"Ambiguous DDR reservations");base=b;}
        }directory.reset();require(base!=0,"No suitable named no-map DDR reservation below 2 GiB");
        std::ifstream iomem("/proc/iomem");std::string line;int ram_regions=0;
        while(std::getline(iomem,line)){unsigned long long lo=0,hi=0;
            if(line.find("System RAM")!=std::string::npos&&std::sscanf(line.c_str()," %llx-%llx",&lo,&hi)==2){
                require(hi>lo,"Redacted/unusable /proc/iomem");++ram_regions;
                require(!(lo<base+BYTES&&hi>=base),"Reserved DDR overlaps System RAM");}}
        require(ram_regions>0,"Cannot verify DDR exclusion from System RAM");
        fd=open("/dev/mem",O_RDWR|O_SYNC);require(fd>=0,"Cannot open /dev/mem");
        mapping=mmap(nullptr,BYTES,PROT_READ|PROT_WRITE,MAP_SHARED,fd,base);require(mapping!=MAP_FAILED,"Cannot map reserved DDR");
        void* r=mmap(nullptr,4096,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0xA1000000ull);require(r!=MAP_FAILED,"Cannot map full-model control");regs=static_cast<volatile uint32_t*>(r);
        void* g=mmap(nullptr,4096,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0xff0a0000ull);require(g!=MAP_FAILED,"Cannot map PL emergency reset");gpio=static_cast<volatile uint32_t*>(g);
        job.probe();
        static_assert(ps_layout::words(N)*sizeof(ps_layout::Pair)<=KTO-KO,"Matrix slot overflow");
        PS_SCOPE(profile_pack, MatrixPack);
        ps_layout::pack_into(m.k.data(),m.k.size(),reinterpret_cast<ps_layout::Pair*>(static_cast<char*>(mapping)+KO),ps_layout::words(N),N);
        ps_layout::pack_into(m.kt.data(),m.kt.size(),reinterpret_cast<ps_layout::Pair*>(static_cast<char*>(mapping)+KTO),ps_layout::words(N),N);barrier();
        PS_STOP(profile_pack);
        pointer(0x1c,base+XO);pointer(0x28,base+YO);
        std::cout<<"PL_DDR_BASE="<<std::hex<<base<<std::dec<<std::endl;
    }catch(...){cleanup();throw;}}
    bool asynchronous()const override{return true;}
    void begin(bool transpose,const Vec& x)override{PS_SCOPE(profile_submit, Submit);job.begin(transpose,x);}
    void finish(Vec& y)override{PS_SCOPE(profile_wait, Wait);job.finish(y);}
    void multiply(bool transpose,const Vec& x,Vec& y)override{begin(transpose,x);finish(y);}
    void cancel()noexcept override{job.stop();}
    ~PL(){cleanup();}
};
}
#else
namespace ps {struct PL:Backend {explicit PL(const Model&){throw std::runtime_error("PL requires AArch64 Linux");}void multiply(bool,const Vec&,Vec&)override{}};}
#endif
