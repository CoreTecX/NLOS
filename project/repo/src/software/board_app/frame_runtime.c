#if defined(__linux__)
#define _POSIX_C_SOURCE 200809L
#include "frame_pipeline.h"
#include "board_pmem.h"
#include "ps_pl_regs.h"
#include <sys/ioctl.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>


/* Explicit fixed reserved-memory backend for vendor kernels without u-dma-buf.
 * Verify no-map DT reservation and exclusion from EVERY System RAM region.
 * O_SYNC /dev/mem maps this excluded range as device memory on ARM64.
 * The backing storage persists after exit; never allocate Linux-owned DDR. */
static int reserved_ddr_env(ps_pmem_env_t *out,const ps_pmem_env_t *base) {
    const char *node="/proc/device-tree/reserved-memory/penumbrasight_dma@40000000";
    char path[160],line[256];unsigned char reg[16];size_t n;FILE *f;
    unsigned long long lo,hi;int ram_regions=0;
    snprintf(path,sizeof(path),"%s/no-map",node);
    if(access(path,F_OK))return -1;
    snprintf(path,sizeof(path),"%s/reg",node);
    f=fopen(path,"rb");if(!f)return -1;n=fread(reg,1,16,f);fclose(f);
    {const unsigned char expected[16]={0,0,0,0,0x40,0,0,0,0,0,0,0,0x20,0,0,0};
     if(n!=16||memcmp(reg,expected,16))return -1;}
    f=fopen("/proc/iomem","r");if(!f)return -1;
    while(fgets(line,sizeof(line),f)) {
        if(strstr(line,"System RAM")&&sscanf(line," %llx-%llx",&lo,&hi)==2) {
           ++ram_regions;
           if(lo<0x60000000ull&&hi>=0x40000000ull){fclose(f);return -1;}
        }
    }
    fclose(f);if(!ram_regions)return -1;
    *out=*base;out->devmem_base=0x40000000ull;out->devmem_bytes=0x20000000ull;
    return 0;
}

typedef struct {
    ps_fix_window_t window;
    ps_fix_pipeline_t cores;
    ps_pmem_block_t memory;
    int cpu_owned,hardware_valid;
    uint64_t timeout_ns;
} runtime_t;
static uint64_t now_ns(void) {
    struct timespec ts;
    if(clock_gettime(CLOCK_MONOTONIC,&ts))return 0;
    return (uint64_t)ts.tv_sec*1000000000ull+(uint64_t)ts.tv_nsec;
}
static uint64_t core_clock(void *ctx) {(void)ctx;return now_ns();}
static volatile uint32_t *pl(runtime_t *r) {return r->window.window+(PS_PL_BASE-r->window.window_phys)/4;}
static int sync_memory(void *ctx,int cpu) {
    runtime_t *r=(runtime_t *)ctx;
    uint64_t flags;
    if(cpu==r->cpu_owned)return 0;
    if(r->memory.kind==PS_PMEM_KIND_DMAHEAP) {
        flags=3ull | (cpu?0ull:4ull); /* DMA_BUF_SYNC_RW + START/END. */
        if(ioctl(r->memory.fd,_IOW('b',0,uint64_t),&flags))return -1;
    } else if(r->memory.kind!=PS_PMEM_KIND_UDMABUF&&r->memory.kind!=PS_PMEM_KIND_DEVMEM)return -1;
    __sync_synchronize();
    r->cpu_owned=cpu;
    return 0;
}
static int wait_dma(runtime_t *r,volatile uint32_t *d,uint32_t status) {
    uint64_t deadline=now_ns()+r->timeout_ns;
    while(now_ns()<deadline) {
        uint32_t s=d[status/4];
        if(s&0x70u)return -1;
        if(s&0x1000u){d[status/4]=0x7000u;__sync_synchronize();return 0;}
    }
    return -1;
}
static int reset_dma(runtime_t *r,volatile uint32_t *d) {
    uint64_t deadline=now_ns()+r->timeout_ns;
    d[0]=4;__sync_synchronize();
    while(d[0]&4){if(now_ns()>deadline)return -1;}
    d[0x04/4]=0x7000;d[0x34/4]=0x7000;
    return 0;
}
static int preprocess(void *ctx,uint64_t raw,uint32_t raw_bytes,uint64_t diff,
                      uint32_t diff_bytes,uint32_t group) {
    runtime_t *r=(runtime_t *)ctx;
    uint32_t count=group>>16,index=group&65535;
    volatile uint32_t *d=r->window.window+0x80000/4,*regs=pl(r);
    if(raw_bytes!=7372800u||diff_bytes!=109440u)return -1;
    if(index==0) {
        if(reset_dma(r,d))return -1;
        regs[PS_PL_RAW_CFG0/4]=PS_PL_RAW_CFG0_MAKE(0,count,0);
        regs[PS_PL_RAW_CFG1/4]=1920;
        regs[PS_PL_DIFF_CFG0/4]=PS_PL_DIFF_CFG0_MAKE(0,96);
        d[0x30/4]=1;
        d[0x4c/4]=(uint32_t)(diff>>32);d[0x48/4]=(uint32_t)diff;
        __sync_synchronize();d[0x58/4]=diff_bytes/4*3;
    }
    d[0]=1;d[0x1c/4]=(uint32_t)(raw>>32);d[0x18/4]=(uint32_t)raw;
    __sync_synchronize();d[0x28/4]=raw_bytes;
    if(wait_dma(r,d,0x04))return -1;
    if(index+1==count) {

        uint8_t *packed=(uint8_t *)r->memory.virt+(diff-r->memory.phys);
        if(wait_dma(r,d,0x34)||d[0x58/4]!=diff_bytes/4*3)return -1;
        if(regs[PS_PL_STAT_STICKY/4]&31u||sync_memory(r,1))return -1;
        /* AXIS 24->32 width conversion packs bytes; it does not pad each
         * 24-bit sample. Expand backwards in the larger arena buffer. */
        if(ps_frame_expand_diff24(packed,diff_bytes,diff_bytes/4))return -1;
    }
    return 0;
}
static int projection(void *ctx,const ps_fix_proj_job_t *j,ps_fix_proj_result_t *out) {
    runtime_t *r=(runtime_t *)ctx;
    int rc=ps_fix_run_projection(&r->cores,j,UINT32_MAX,out);
    if(!rc && (int32_t)r->cores.cores[PS_FIX_CORE_ID_PROJ].ctrl[PS_FIX_REG_AP_RETURN/4])return -1;
    return rc;
}
static int reconstruct(void *ctx,const ps_fix_pdhg_job_t *j,ps_fix_pdhg_result_t *out) {
    runtime_t *r=(runtime_t *)ctx;
    int rc=ps_fix_run_pdhg(&r->cores,j,UINT32_MAX,out);
    if(!rc && (int32_t)r->cores.cores[PS_FIX_CORE_ID_PDHG].ctrl[PS_FIX_REG_AP_RETURN/4])return -1;
    return rc;
}
static int parse_u32(const char *s,uint32_t *out) {
    char *end;unsigned long long x;
    if(!s||!*s||*s=='-'||*s=='+')return -1;
    x=strtoull(s,&end,0);if(*end||x>UINT32_MAX)return -1;*out=(uint32_t)x;return 0;
}

/* Returns -2 when not selected; otherwise an actual process exit status. */
int ps_frame_command(int argc,char **argv) {
    const char *model=NULL,*prefix="result",*uio="auto",*fbdev=NULL,*paths[8];
    uint32_t count=0,hash=0,iters=64,nx=9,ny=9,nz=3,snr=0;
    int legacy=1,i,rc=1,reserved=0,diagnostic=0;ps_pmem_env_t reserved_env;double lambda=66.0/65536;
    runtime_t r;ps_board_arena_t arena;ps_frame_pipeline_t p;ps_frame_backend_t b;
    uint32_t wall[9216];
    char error[256];const ps_pmem_env_t *env=ps_pmem_env_default();
    for(i=1;i<argc;++i)if(!strcmp(argv[i],"--process-model"))break;
    if(i==argc)return -2;
    memset(&r,0,sizeof(r));memset(&p,0,sizeof(p));r.memory.fd=-1;r.timeout_ns=30000000000ull;
    for(i=1;i<argc;++i) {
        const char *key=argv[i];
        if(i+1>=argc){fprintf(stderr,"missing value for %s\n",key);return 2;}
        if(!strcmp(key,"--process-model"))model=argv[++i];
        else if(!strcmp(key,"--frame")){if(count==8)return 2;paths[count++]=argv[++i];}
        else if(!strcmp(key,"--geometry-hash")){if(parse_u32(argv[++i],&hash))return 2;}
        else if(!strcmp(key,"--output"))prefix=argv[++i];
        else if(!strcmp(key,"--uio"))uio=argv[++i];
        else if(!strcmp(key,"--fbdev"))fbdev=argv[++i];
        else if(!strcmp(key,"--reserved-ddr")){if(strcmp(argv[++i],"dt-no-map"))return 2;reserved=1;}
        else if(!strcmp(key,"--diagnostic-preview")){if(strcmp(argv[++i],"yes"))return 2;diagnostic=1;}
        else if(!strcmp(key,"--timeout-ms")){uint32_t ms;if(parse_u32(argv[++i],&ms)||ms<1||ms>600000)return 2;r.timeout_ns=(uint64_t)ms*1000000;}
        else if(!strcmp(key,"--iterations")){if(parse_u32(argv[++i],&iters))return 2;}
        else if(!strcmp(key,"--snr-q16")){if(parse_u32(argv[++i],&snr)||snr>65535)return 2;}
        else if(!strcmp(key,"--lambda-kernel")){char *end;lambda=strtod(argv[++i],&end);if(*end||!isfinite(lambda)||lambda<0||lambda>1)return 2;}
        else if(!strcmp(key,"--model-layout")){const char *v=argv[++i];if(!strcmp(v,"legacy_f"))legacy=1;else if(!strcmp(v,"canonical_c"))legacy=0;else return 2;}
        else {fprintf(stderr,"unknown frame option %s\n",key);return 2;}
    }
    if(!hash||!model||(count!=1&&count!=4&&count!=8)||iters<8||iters>4096){fprintf(stderr,"require model, geometry hash, 1/4/8 frames and 8..4096 iterations\n");return 2;}
    if(ps_fix_window_open_uio(&r.window,uio,error,sizeof(error))){fprintf(stderr,"%s\n",error);return 3;}
    if(r.window.window_bytes<0xB0000u||pl(&r)[PS_PL_ID/4]!=0x50530001u||pl(&r)[PS_PL_VERSION/4]!=PS_FIX_IF_VERSION||pl(&r)[PS_PL_CE_START/4]!=1){
        fprintf(stderr,"UIO must cover cores, DMA and control page (1 MiB), correct bitstream required\n");goto cleanup;
    }
    /* 512 MiB covers B-profile 300 MiB model + packed U and scratch buffers. */
    if(reserved) {
        if(reserved_ddr_env(&reserved_env,env)){fprintf(stderr,"fixed DDR no-map reservation missing or overlaps System RAM\n");goto cleanup;}
        env=&reserved_env;
    }
    if(ps_pmem_alloc(env,512ull*1024*1024,PS_PMEM_F_REQUIRE_PHYS|(reserved?PS_PMEM_F_ALLOW_DEVMEM:PS_PMEM_F_REQUIRE_UDMABUF),&r.memory,error,sizeof(error))){fprintf(stderr,"%s\n",error);goto cleanup;}
    /* HLS has no userspace abort/reset. A timed-out master may still own DDR.
     * Only persistent reserved udmabuf is safe across failure/exit. */
    if(r.memory.kind!=PS_PMEM_KIND_UDMABUF && !(reserved&&r.memory.kind==PS_PMEM_KIND_DEVMEM)) {fprintf(stderr,"frame runtime requires persistent reserved udmabuf; heap timeout cannot be safely reclaimed\n");goto cleanup;}
    if(flock(r.memory.fd,LOCK_EX|LOCK_NB)) {fprintf(stderr,"reserved udmabuf already in use\n");goto cleanup;}
    r.hardware_valid=1;
    if(fbdev&&ps_frame_present_fbdev(NULL,NULL,fbdev,1)) {fprintf(stderr,"cannot clear requested framebuffer\n");goto cleanup;}
    pl(&r)[PS_PL_CMP_CTRL/4]=1;
    ps_pl_set_geom_hash_ok(pl(&r),0); /* fail closed until valid result. */
    if(pl(&r)[PS_PL_STAT_STICKY/4]&31u) {fprintf(stderr,"RAW pipeline error sticky: reload FPGA\n");goto cleanup;}
    if(r.memory.phys>=0x7ff00000ull || r.memory.bytes>0x7ff00000ull-r.memory.phys) {
        fprintf(stderr,"reserved DDR must fit low DDR aperture below PMU reserved range\n");goto cleanup;
    }
    if(ps_fix_pipeline_open(&r.cores,&r.window))goto cleanup;
    for(i=0;i<4;++i) {
        if(!(r.cores.cores[i].ctrl[PS_FIX_REG_CTRL/4]&PS_FIX_CTRL_AP_IDLE)) {
            fprintf(stderr,"HLS core busy: reload FPGA after prior timeout\n");goto cleanup;
        }
        r.cores.cores[i].clock_ns=core_clock;r.cores.cores[i].timeout_ns=r.timeout_ns;
    }
    r.cpu_owned=0;
    if(sync_memory(&r,1)||ps_board_arena_init(&arena,r.memory.phys,r.memory.bytes,r.memory.virt))goto cleanup;
    fprintf(stderr,"STAGE=MODEL_LOAD\n");
    if(ps_frame_init(&p,&arena,model,hash,nx,ny,nz,legacy)){fprintf(stderr,"%s\n",p.error);goto cleanup;}
    fprintf(stderr,"STAGE=MODEL_READY\n");
    p.iterations=iters;p.lambda_q16=(int32_t)nearbyint(lambda*65536);
    b.ctx=&r;b.preprocess=preprocess;b.projection=projection;b.reconstruction=reconstruct;b.sync=sync_memory;
    {uint64_t started=now_ns();
        if(ps_frame_process(&p,paths,count,&b)){fprintf(stderr,"frame failed: %s\n",p.error);goto cleanup;}
        printf("PROCESS_NS=%llu\n",(unsigned long long)(now_ns()-started));
    }
    /* Bridge the CURRENT software results; uncalibrated SNR stays zero, not guessed. */
    ps_pl_ce_set_margin(pl(&r),(uint32_t)p.localization.margin_q16);
    {uint32_t rho=p.channels.rho_q16[0];for(i=1;i<3;++i)if(p.channels.rho_q16[i]>rho)rho=p.channels.rho_q16[i];
        ps_pl_ce_set_rho(pl(&r),rho>65535?65535:rho);}
    pl(&r)[PS_PL_CE_SNR/4]=snr;
    if(ps_frame_publish_display(&p,pl(&r),wall)){fprintf(stderr,"display publication failed\n");goto cleanup;}
    ps_pl_set_geom_hash_ok(pl(&r),1);
    pl(&r)[PS_PL_CMP_SELID/4]=p.selected_id;
    pl(&r)[PS_PL_CMP_X/4]=p.selected_id/(ny*nz);
    pl(&r)[PS_PL_CMP_Y/4]=(p.selected_id/nz)%ny;pl(&r)[PS_PL_CMP_Z/4]=p.selected_id%nz;
    {uint32_t previous=pl(&r)[PS_PL_CTR_CE/4];uint64_t started=now_ns();
        __sync_synchronize();pl(&r)[PS_PL_CE_START/4]=1;
        while(pl(&r)[PS_PL_CTR_CE/4]==previous) {
            if(now_ns()-started>=r.timeout_ns){fprintf(stderr,"confidence engine timeout\n");goto cleanup;}
        }
    }
    pl(&r)[PS_PL_CMP_CONF/4]=pl(&r)[PS_PL_CE_CONF/4];
    pl(&r)[PS_PL_CMP_CTRL/4]=p.localization.low_conf;
    if(fbdev&&ps_frame_present_fbdev(&p,wall,fbdev,diagnostic?2:(p.localization.low_conf||
            (pl(&r)[PS_PL_CE_STAT/4]&3u)))) {fprintf(stderr,"unsupported framebuffer geometry or pixel layout\n");goto cleanup;}
    printf("CONFIDENCE_STAT=%u\nDIAGNOSTIC_PREVIEW=%d\n",pl(&r)[PS_PL_CE_STAT/4],diagnostic);
    if(ps_frame_write_result(&p,prefix)){fprintf(stderr,"result write failed\n");goto cleanup;}
    printf("SELECTED_ID=%u\nDDR_USED=%llu\nOUTPUT=%s\nHARDWARE_VALIDATION=REQUIRED\n",p.selected_id,(unsigned long long)arena.used,prefix);
    rc=0;
cleanup:
    if(rc&&fbdev&&r.hardware_valid)(void)ps_frame_present_fbdev(NULL,NULL,fbdev,1);
    if(rc && r.hardware_valid){ps_pl_set_geom_hash_ok(pl(&r),0);pl(&r)[PS_PL_CMP_CTRL/4]=1;}
    if(r.hardware_valid && r.window.window_bytes>=0xA0000u) {
        (void)reset_dma(&r,r.window.window+0x80000/4);
        (void)reset_dma(&r,r.window.window+0x90000/4);
    }
    if(r.memory.virt){(void)sync_memory(&r,0);ps_pmem_free(env,&r.memory);}
    ps_fix_window_close(&r.window);
    return rc;
}
#endif
