#include "frame_pipeline.h"
#include "ps_fixed_top.hpp"
#include "ps_pl_regs.h"
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>

struct Host {ps_frame_pipeline_t *p;std::vector<int64_t> blocks;};
template<class T> static T *ptr(Host *h,uint64_t phys){return static_cast<T *>(ps_frame_ptr(h->p,phys));}
static int sync_host(void*,int){return 0;}
static int raw_host(void *ctx,uint64_t src,uint32_t,uint64_t dst,uint32_t,uint32_t group){
    Host *h=static_cast<Host *>(ctx);const uint16_t *raw=ptr<uint16_t>(h,src);
    unsigned frames=group>>16,index=group&65535;
    if(index==0)h->blocks.assign(96*96*3,0);
    for(int by=0;by<96;++by)for(int bx=0;bx<96;++bx){
        int64_t a[3]={0,0,0};
        for(int y=0;y<20;++y)for(int x=0;x<20;++x){
            unsigned c=((y&1)==(x&1))?((y&1)?2:0):1;
            a[c]+=raw[(by*20+y)*1920+bx*20+x];
        }
        /* RTL averages the two greens AFTER block accumulation, half-to-even. */
        int64_t g=a[1]/2;if((a[1]&1)&&(g&1))++g;a[1]=g;
        for(int c=0;c<3;++c)h->blocks[(by*96+bx)*3+c]+=a[c];
    }
    if(index+1==frames){uint8_t *out=ptr<uint8_t>(h,dst);
        for(int y=0;y<95;++y)for(int x=0;x<96;++x)for(int c=0;c<3;++c){
            int64_t d=h->blocks[((y+1)*96+x)*3+c]-h->blocks[(y*96+x)*3+c];
            if(d>8388607||d< -8388607)return 1;
            uint32_t code=static_cast<uint32_t>(d)&0xffffff;
            unsigned i=((y*96+x)*3+c)*3;
            out[i]=code&255;out[i+1]=(code>>8)&255;out[i+2]=(code>>16)&255;
        }
        if(ps_frame_expand_diff24(out,95*96*3*4,95*96*3))return 1;
    }return 0;
}
static int proj_host(void *ctx,const ps_fix_proj_job_t *j,ps_fix_proj_result_t *o){
    Host *h=static_cast<Host *>(ctx);int best,second,margin,bound,low,ov,sat,mac,out,score,gain;long long max;
    unsigned long long bs,ss;
    int rc=ps_projection_fix(ptr<int32_t>(h,j->meas),ptr<int16_t>(h,j->u_codes),ptr<int32_t>(h,j->gain_q16),
        j->P,j->K,j->M,j->nx,j->ny,j->nz,j->margin_thresh_q16,ptr<uint64_t>(h,j->score_out),ptr<uint64_t>(h,j->score_ch),
        &best,&second,&margin,&bound,&low,&ov,&bs,&ss,&sat,&mac,&out,&score,&gain,&max);
    o->best_id=best;o->second_id=second;o->margin_q16=margin;o->boundary=bound;o->low_conf=low;o->overflow=ov;
    o->best_score=bs;o->second_score=ss;o->sat_events=sat;o->mac_overflow=mac;o->out_overflow=out;
    o->score_overflow=score;o->gain_clamped=gain;o->max_abs_acc=max;return rc;
}
static int recon_host(void *ctx,const ps_fix_pdhg_job_t *j,ps_fix_pdhg_result_t *o){
    Host *h=static_cast<Host *>(ctx);int sat,mac,out,score,gain;long long max;
    int rc=ps_pdhg_fix(ptr<int16_t>(h,j->U),ptr<uint16_t>(h,j->S),ptr<int16_t>(h,j->V),j->K,j->M,j->N,
        j->gain.u_gain_q16,j->gain.s_gain_q16,j->gain.v_gain_q16,j->gain.u_exp,j->gain.s_exp,j->gain.v_exp,
        ptr<int32_t>(h,j->d3),j->params.n_rows,j->params.n_cols,j->params.n_iter,j->params.min_iters,
        j->params.residual_every,j->params.tv_mode,j->params.tau_q16,j->params.sigma_q16,j->params.lambda_q16,
        j->params.theta_q16,j->params.rho_stop_q8,ptr<int32_t>(h,j->f3),ptr<int32_t>(h,j->scratch.wk_f_prev),
        ptr<int32_t>(h,j->scratch.wk_fbar),ptr<int32_t>(h,j->scratch.wk_q),ptr<int32_t>(h,j->scratch.wk_grad),
        ptr<int32_t>(h,j->scratch.wk_divq),ptr<int32_t>(h,j->scratch.wk_y),ptr<int32_t>(h,j->scratch.wk_r),
        ptr<int32_t>(h,j->scratch.wk_btr),ptr<int32_t>(h,j->scratch.wk_tmp),ptr<int>(h,j->out.out_n_used),
        ptr<int>(h,j->out.out_early_stopped),ptr<int>(h,j->out.out_overflow_ch),ptr<unsigned int>(h,j->out.out_rho_q16),
        ptr<unsigned long long>(h,j->out.out_res_sq),ptr<unsigned long long>(h,j->out.out_d_sq),
        ptr<long long>(h,j->out.out_mac_count),&sat,&mac,&out,&score,&gain,&max);
    o->sat_events=sat;o->mac_overflow=mac;o->out_overflow=out;o->score_overflow=score;o->gain_clamped=gain;o->max_abs_acc=max;
    return rc;
}
int main(int argc,char **argv){
    {uint32_t storage[5]={0};unsigned char *b=(unsigned char *)storage;
        const uint32_t expected[5]={0,1,0x7fffff,0x800001,0xffffff};
        for(unsigned i=0;i<5;++i)for(unsigned j=0;j<3;++j)b[3*i+j]=(expected[i]>>(8*j))&255;
        if(ps_frame_expand_diff24(storage,sizeof(storage)-1,5)==0)return 14;
        if(ps_frame_expand_diff24(storage,sizeof(storage),5))return 15;
        for(unsigned i=0;i<5;++i)if(storage[i]!=expected[i])return 16;
    }

    if(argc==5 && !std::strcmp(argv[1],"--production")) {
        std::vector<unsigned char> memory(512ull*1024*1024);ps_board_arena_t arena;ps_frame_pipeline_t p;
        if(ps_board_arena_init(&arena,0x20000000,memory.size(),memory.data()))return 3;
        if(ps_frame_init(&p,&arena,argv[2],3552567744u,9,9,3,1))return 4;
        p.iterations=64;p.lambda_q16=66;Host h;h.p=&p;
        ps_frame_backend_t b={&h,raw_host,proj_host,recon_host,sync_host};const char *paths[]={argv[3]};
        if(ps_frame_process(&p,paths,1,&b))return 5;
        if(ps_frame_write_result(&p,argv[4]))return 8;
        std::printf("PRODUCTION_MODEL_FRAME_PASS candidate=%u rank=%u used=%llu low_conf=%u\n",
            p.selected_id,p.model.rank_default,(unsigned long long)arena.used,p.localization.low_conf);
        return 0;
    }
    if(argc<4||argc>11){std::fprintf(stderr,"model.psm prefix frame.psf [frame...]\n");return 2;}
    std::vector<unsigned char> memory(64*1024*1024);ps_board_arena_t arena;ps_frame_pipeline_t p;
    if(ps_board_arena_init(&arena,0x20000000,memory.size(),memory.data()))return 3;
    if(ps_frame_init(&p,&arena,argv[1],0x13572468,3,1,1,1)){std::fprintf(stderr,"%s\n",p.error);return 4;}
    p.iterations=128;p.lambda_q16=0;
    Host h;h.p=&p;ps_frame_backend_t b={&h,raw_host,proj_host,recon_host,sync_host};const char *const *paths=(const char *const *)(argv+3);unsigned count=(unsigned)(argc-3);
    if(ps_frame_process(&p,paths,count,&b)){std::fprintf(stderr,"%s\n",p.error);return 5;}
    if(p.selected_id!=1||p.channels.n_used[0]!=128||p.channels.overflow_ch[0])return 6;
    const int32_t *im=ptr<int32_t>(&h,p.image);
    int32_t max=*std::max_element(im,im+432);
    if(max<=0||im[2*24+3]!=max){std::fprintf(stderr,"hidden F->C conversion failed\n");return 7;}
    if(ps_frame_write_result(&p,argv[2]))return 8;
    std::vector<uint32_t> regs(1024),wall(9216);
    if(ps_frame_publish_display(&p,regs.data(),wall.data()))return 12;
    if(wall[0]!=0x7f7f7fu||wall[96]!=0xffffffu||regs[PS_PL_CMP_REC_ADDR/4]!=431||
       regs[PS_PL_MASK_ADDR/4]!=431||regs[PS_PL_MASK_DATA/4]!=255)return 13;
    /* Bad hash, bad frame count and fake-address-free binding are hard gates. */
    const uint64_t bad[3]={p.selected_u,p.selected_v,p.meas};
    for(auto a:bad)if(a<arena.base||a>=arena.base+arena.used)return 9;
    if(ps_frame_process(&p,paths,2,&b)==0||p.completed)return 10;
    if(ps_frame_write_result(&p,argv[2])==0)return 11;
    std::printf("FRAME_E2E_PASS selected=%u pixel=(2,3) max=%d iterations=128 bytes=%llu\n",p.selected_id,max,(unsigned long long)arena.used);
    return 0;
}
