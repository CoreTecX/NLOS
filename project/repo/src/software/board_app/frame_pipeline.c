#include "frame_pipeline.h"
#include "sd_frame.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include <stdlib.h>

int ps_frame_expand_diff24(void *buffer,size_t capacity_bytes,uint32_t samples) {
    uint8_t *bytes=(uint8_t *)buffer;uint32_t *words=(uint32_t *)buffer;
    uint32_t i=samples;
    if(!buffer||((uintptr_t)buffer&3u)||(uint64_t)samples*4>capacity_bytes)return -1;
    while(i--) {
        uint32_t code=(uint32_t)bytes[3*i]|((uint32_t)bytes[3*i+1]<<8)|((uint32_t)bytes[3*i+2]<<16);
        words[i]=code;
    }
    return 0;
}

static int fail(ps_frame_pipeline_t *p, const char *message) {
    snprintf(p->error, sizeof(p->error), "%s", message);
    p->completed = 0;
    return -1;
}
void *ps_frame_ptr(const ps_frame_pipeline_t *p, uint64_t phys) {
    return ps_board_arena_ptr(p->arena, phys);
}
static int alloc(ps_frame_pipeline_t *p, uint64_t bytes, uint64_t *phys) {
    return ps_board_arena_alloc(p->arena, bytes, phys);
}
static const uint8_t *record(const ps_frame_pipeline_t *p, uint32_t index) {
    return (const uint8_t *)ps_frame_ptr(p, p->records) +
        (uint64_t)index * ps_psm_record_bytes_hdr(&p->model);
}
static int load_model(ps_frame_pipeline_t *p, const char *path, uint32_t hash) {
    FILE *f = fopen(path, "rb");
    uint64_t done = 0;
    uint32_t crc = 0;
    uint8_t *dst, *buffer;
    if (!f) return fail(p, "cannot open model");
    if (fread(&p->model, 1, sizeof(p->model), f) != sizeof(p->model) ||
        !hash || ps_psm_check_header(&p->model, hash, 95, 96, 18, 24) != PS_PSM_OK) {
        fclose(f); return fail(p, "invalid model header, geometry or B-profile shape");
    }
    if ((uint64_t)p->nx * p->ny * p->nz != p->model.candidates ||
        p->model.loc_rank > 8 || p->model.rank_default > 96 ||
        p->model.rank_max > 96) {
        fclose(f); return fail(p, "model exceeds frozen hardware or grid contract");
    }
    if (alloc(p, p->model.payload_bytes, &p->records)) {
        fclose(f); return fail(p, "DDR arena too small for model payload");
    }
    dst = (uint8_t *)ps_frame_ptr(p, p->records);
    buffer=(uint8_t *)malloc(1048576);
    if(!buffer){fclose(f);return fail(p,"model staging allocation failed");}
    while (done < p->model.payload_bytes) {
        size_t n = (size_t)((p->model.payload_bytes - done > 1048576) ? 1048576 : p->model.payload_bytes - done);
        if (fread(buffer, 1, n, f) != n) { free(buffer); fclose(f); return fail(p, "truncated model"); }
        crc = ps_crc32(crc, buffer, n);
        memcpy(dst+done,buffer,n);
        done += n;
    }
    free(buffer);
    if (fgetc(f) != EOF || ferror(f) || crc != p->model.payload_crc32) {
        fclose(f); return fail(p, "model CRC or file length mismatch");
    }
    fclose(f);
    return 0;
}

int ps_frame_init(ps_frame_pipeline_t *p, ps_board_arena_t *arena, const char *path,
                  uint32_t hash, uint32_t nx, uint32_t ny, uint32_t nz, int legacy) {
    uint64_t m, n, k, count, rank, uoff;
    uint32_t i;
    if (!p || !arena || !arena->host || !path || !nx || !ny || !nz) return -1;
    memset(p, 0, sizeof(*p));
    p->arena = arena; p->nx = nx; p->ny = ny; p->nz = nz; p->legacy_layout = legacy;
    p->iterations = 64; p->lambda_q16 = 66;
    if (load_model(p, path, hash)) return -1;
    m = ps_psm_meas_len(&p->model); n = ps_psm_hidden_len(&p->model);
    k = p->model.loc_rank; rank = p->model.rank_default; count = p->model.candidates;
#define AL(field, bytes) if (alloc(p, (bytes), &p->field)) return fail(p, "DDR arena exhausted")
    AL(loc_u, count*k*m*2); AL(gains, count*4);
    AL(raw, 1920ull*1920*2); AL(diff, m*3*4); AL(meas, m*3*4);
    AL(scores, count*8); AL(score_ch, count*3*8);
    AL(selected_u, rank*m*2); AL(selected_s, rank*2); AL(selected_v, rank*n*2);
    AL(image, 3*n*4);
    AL(scratch.wk_f_prev, n*4); AL(scratch.wk_fbar, n*4);
    AL(scratch.wk_q, 2*n*4); AL(scratch.wk_grad, 2*n*4); AL(scratch.wk_divq, n*4);
    AL(scratch.wk_y, m*4); AL(scratch.wk_r, m*4); AL(scratch.wk_btr, n*4); AL(scratch.wk_tmp, n*4);
    AL(diagnostics.out_n_used, 12); AL(diagnostics.out_early_stopped, 12);
    AL(diagnostics.out_overflow_ch, 12); AL(diagnostics.out_rho_q16, 12);
    AL(diagnostics.out_res_sq, 24); AL(diagnostics.out_d_sq, 24); AL(diagnostics.out_mac_count, 24);
#undef AL
    uoff = ps_psm_u_offset(p->model.rank_max, m);
    for (i = 0; i < count; ++i) {
        const uint8_t *rec = record(p, i);
        ps_fix_gain_t gain;
        uint32_t j, z, x;
        int16_t *out = (int16_t *)ps_frame_ptr(p, p->loc_u) + (uint64_t)i*k*m;
        if (ps_fix_gain_from_candidate(rec, &gain) != PS_FIX_OK) return fail(p, "invalid candidate scales");
        /* Measurement F -> C is a row permutation; singular values unchanged. */
        for (j=0; j<k; ++j) for (z=0; z<p->model.meas_h; ++z) for (x=0; x<p->model.meas_w; ++x) {
            uint64_t idx = legacy ? (uint64_t)x*p->model.meas_h+z : (uint64_t)z*p->model.meas_w+x;
            out[(uint64_t)j*m+(uint64_t)z*p->model.meas_w+x] = ps_psm_read_i16(rec, uoff+((uint64_t)j*m+idx)*2);
        }
    }
    p->initialized = 1;
    return 0;
}

static int normalize_and_gains(ps_frame_pipeline_t *p) {
    uint32_t i;
    double bound, max_u=0;
    int exponent;
    int32_t *g = (int32_t *)ps_frame_ptr(p, p->gains);
    if (ps_fix_prepare_meas((const uint32_t *)ps_frame_ptr(p,p->diff),p->model.meas_h,
            p->model.meas_w,3,(int32_t *)ps_frame_ptr(p,p->meas),&p->measurement))
        return fail(p,"measurement normalization failed");
    if (!p->measurement.raw_absmax) return fail(p,"empty differential signal");
    for (i=0;i<p->model.candidates;++i) {
        double u=ps_psm_read_f32(record(p,i),8);
        if (u>max_u) max_u=u;
    }
    /* Common power-of-two score normalization keeps every candidate comparable,
     * and makes candidate Q16 gains exact. Absolute exposure scale cancels in ranking. */
    bound = 1073741824.0 / (p->measurement.code_absmax*sqrt((double)ps_psm_meas_len(&p->model)));
    if (bound > 65536.0/(max_u*65536.0)) bound = 1.0/max_u;
    exponent=(int)floor(log2(bound));
    for (i=0;i<p->model.candidates;++i) {
        double code=ldexp((double)ps_psm_read_f32(record(p,i),8),exponent+16);
        if (code<1 || code>65536 || code!=floor(code)) return fail(p,"projection gain cannot be represented exactly");
        g[i]=(int32_t)code;
    }
    return 0;
}

/* Rigorous spectral upper bound from ||Q Q^T||_infinity. Validate the
 * quantized matrices themselves: s_max metadata alone is not a norm proof.
 * Selected model only, once per reconstruction; int64 dot products are exact. */
static double gram_norm_bound(const int16_t *q,uint32_t k,uint32_t len,double scale) {
    uint32_t i,j,t;double best=0;
    for(i=0;i<k;++i) {
        double row=0;
        for(j=0;j<k;++j) {
            int64_t dot=0;
            for(t=0;t<len;++t) dot+=(int64_t)q[(uint64_t)i*len+t]*q[(uint64_t)j*len+t];
            row+=fabs((double)dot)*scale*scale;
        }
        if(row>best)best=row;
    }
    return sqrt(best);
}

static int bind_reconstruction(ps_frame_pipeline_t *p, ps_fix_pdhg_job_t *job) {
    uint64_t m=ps_psm_meas_len(&p->model),n=ps_psm_hidden_len(&p->model);
    uint32_t k=p->model.rank_default,j,z,x;
    const uint8_t *rec=record(p,p->selected_id);
    int16_t *u=(int16_t *)ps_frame_ptr(p,p->selected_u), *v=(int16_t *)ps_frame_ptr(p,p->selected_v);
    uint16_t *s=(uint16_t *)ps_frame_ptr(p,p->selected_s);
    uint64_t uoff=ps_psm_u_offset(p->model.rank_max,m),voff=ps_psm_v_offset(p->model.rank_max,m);

    double smax=ps_psm_read_s_max(rec),sscale=ps_psm_read_f32(rec,12),norm;
    memset(job,0,sizeof(*job));
    if (!(smax>0) || !isfinite(smax) || ps_fix_gain_from_candidate(rec,&job->gain)) return fail(p,"invalid reconstruction scale");
    for (j=0;j<k;++j) {
        s[j]=ps_psm_read_S_k(rec,&p->model,(uint16_t)j);
        for(z=0;z<p->model.meas_h;++z) for(x=0;x<p->model.meas_w;++x) {
            uint64_t src=p->legacy_layout?(uint64_t)x*p->model.meas_h+z:(uint64_t)z*p->model.meas_w+x;
            u[(uint64_t)j*m+(uint64_t)z*p->model.meas_w+x]=ps_psm_read_i16(rec,uoff+((uint64_t)j*m+src)*2);
        }
        for(z=0;z<p->model.hidden_h;++z) for(x=0;x<p->model.hidden_w;++x) {
            uint64_t src=p->legacy_layout?(uint64_t)x*p->model.hidden_h+z:(uint64_t)z*p->model.hidden_w+x;
            v[(uint64_t)j*n+(uint64_t)z*p->model.hidden_w+x]=ps_psm_read_i16(rec,voff+((uint64_t)j*n+src)*2);
        }
    }
    /* d=raw/2^e, z=f/2^image_exp; B_kernel=B*2^(image_exp-e).
     * Keep ||B_kernel|| in (1,2], instead of changing regularization accidentally. */
    p->image_scale_exp=p->measurement.exponent-(int)ceil(log2(smax))+1;
    /* A tiny non-power-of-two S gain rounded to Q16 may lose >10%.
     * Requantize selected S into uint16 with ONE exact power-of-two multiplier,
     * instead. This preserves full code precision and avoids gain clipping. */
    {double multiplier=ldexp(sscale*smax,p->image_scale_exp-p->measurement.exponent);
        double peak=0;int exp;
        for(j=0;j<k;++j)if(s[j]>peak)peak=s[j];
        if(!isfinite(multiplier)||multiplier<=0||peak<=0)return fail(p,"invalid singular values");
        exp=(int)ceil(log2(peak*multiplier/65535.0));
        if(exp< -128||exp>127)return fail(p,"singular-value exponent out of hardware range");
        for(j=0;j<k;++j) {
            double value=ldexp(s[j]*multiplier,-exp),integer=floor(value),frac=value-integer;
            if(frac>0.5||(frac==0.5&&((uint64_t)integer&1)))integer+=1;
            if(integer<0||integer>65535)return fail(p,"singular-value code overflow");
            s[j]=(uint16_t)integer;
        }
        job->gain.s_exp=exp;job->gain.s_gain_q16=0;
    }
    job->U=p->selected_u; job->S=p->selected_s; job->V=p->selected_v;
    job->d3=p->meas; job->f3=p->image; job->K=k; job->M=(uint32_t)m; job->N=(uint32_t)n;
    job->scratch=p->scratch; job->out=p->diagnostics;
    job->params.n_rows=p->model.hidden_h; job->params.n_cols=p->model.hidden_w;
    job->params.n_iter=p->iterations; job->params.min_iters=8; job->params.residual_every=4;
    job->params.lambda_q16=p->lambda_q16; job->params.theta_q16=65536;
    job->params.sigma_q16=16384;
    /* Smooth-data primal/TV-dual condition: tau*(||B||^2/2+sigma*||grad||^2)<1. */
    {double s_peak=0;for(j=0;j<k;++j)if(s[j]>s_peak)s_peak=s[j];
        norm=gram_norm_bound(u,k,(uint32_t)m,ps_psm_read_f32(rec,8))*
             gram_norm_bound(v,k,(uint32_t)n,ps_psm_read_f32(rec,16))*
             ldexp(s_peak,job->gain.s_exp);
        if(!isfinite(norm)||norm<=0) return fail(p,"zero or invalid quantized operator");
    }
    job->params.tau_q16=(int32_t)floor(0.9/(norm*norm/2+0.25*8)*65536);
    if(job->params.tau_q16<1) return fail(p,"operator too large for a stable fixed-point step");
    job->params.rho_stop_q8=0; /* No residual-based early stop without calibrated threshold. */
    return 0;
}

int ps_frame_process(ps_frame_pipeline_t *p,const char *const *paths,uint32_t count,
                     const ps_frame_backend_t *b) {
    uint32_t i,last_index=0;
    ps_fix_proj_job_t proj;
    ps_fix_pdhg_job_t recon;
    uint32_t bytes;
    if (p) p->completed=0;
    if (!p||!p->initialized||!paths||!b||!b->preprocess||!b->projection||!b->reconstruction||
        !b->sync||(count!=1&&count!=4&&count!=8)||p->iterations<8||p->iterations>4096||
        p->lambda_q16<0||p->lambda_q16>65536) return -1;
    bytes=(uint32_t)(ps_psm_meas_len(&p->model)*3*4);
    p->completed=0;p->error[0]=0;p->frames=count;
    /* Validate the whole immutable input group before issuing any DMA. A bad
     * late frame must not strand RAW accumulation half way through a group. */
    if(b->sync(b->ctx,1))return fail(p,"CPU memory ownership unavailable");
    for(i=0;i<count;++i) {
        ps_sd_frame_header_t h;ps_sd_frame_check_t check;
        if(ps_sd_frame_read_file(paths[i],&h,(uint16_t *)ps_frame_ptr(p,p->raw),1920u*1920,&check,p->error,sizeof(p->error)))return -1;
        if(h.width!=1920||h.height!=1920||h.valid_bits!=12||h.value_packing!=PS_SD_PACKING_PACKED||
            h.bayer_phase!=PS_SD_BAYER_RGGB||h.payload_max>4095||
            (i&&h.frame_index!=last_index+1))return fail(p,"RAW geometry, packing, Bayer or frame continuity mismatch");
        if(i && (h.scale_num!=p->input_scale_num||h.scale_den!=p->input_scale_den))
            return fail(p,"frame group has inconsistent quantization scales");
        if(!i){p->input_scale_num=h.scale_num;p->input_scale_den=h.scale_den;}
        last_index=h.frame_index;
    }
    last_index=0;
    for(i=0;i<count;++i) {
        ps_sd_frame_header_t h; ps_sd_frame_check_t check;
        if(b->sync(b->ctx,1)) return fail(p,"CPU memory ownership unavailable");
        if(ps_sd_frame_read_file(paths[i],&h,(uint16_t *)ps_frame_ptr(p,p->raw),1920u*1920,&check,p->error,sizeof(p->error))) return -1;
        if(h.width!=1920||h.height!=1920||h.valid_bits!=12||h.value_packing!=PS_SD_PACKING_PACKED||
            h.bayer_phase!=PS_SD_BAYER_RGGB||h.payload_max>4095||
            (i&&h.frame_index!=last_index+1)) return fail(p,"RAW geometry, packing, Bayer or frame continuity mismatch");
        if(i && (h.scale_num!=p->input_scale_num||h.scale_den!=p->input_scale_den))
            return fail(p,"frame group has inconsistent quantization scales");
        if(!i){p->input_scale_num=h.scale_num;p->input_scale_den=h.scale_den;}
        last_index=h.frame_index;
        if(b->sync(b->ctx,0)||b->preprocess(b->ctx,p->raw,h.payload_bytes,p->diff,bytes,(count<<16)|i)) return fail(p,"RAW DMA failed");
    }
    if(b->sync(b->ctx,1)||normalize_and_gains(p)) return -1;
    memset(&proj,0,sizeof(proj));
    proj.meas=p->meas;proj.u_codes=p->loc_u;proj.gain_q16=p->gains;
    proj.score_out=p->scores;proj.score_ch=p->score_ch;
    proj.P=p->model.candidates;proj.K=p->model.loc_rank;proj.M=(uint32_t)ps_psm_meas_len(&p->model);
    proj.nx=p->nx;proj.ny=p->ny;proj.nz=p->nz;proj.margin_thresh_q16=16384;
    if(b->sync(b->ctx,0)||b->projection(b->ctx,&proj,&p->localization)||b->sync(b->ctx,1)) return fail(p,"projection failed");
    if(p->localization.best_id>=p->model.candidates||p->localization.overflow||
       p->localization.sat_events||p->localization.mac_overflow||p->localization.out_overflow||
       p->localization.score_overflow||p->localization.gain_clamped) return fail(p,"invalid or overflowing projection");
    p->selected_id=p->localization.best_id;
    if(bind_reconstruction(p,&recon)) return -1;
    if(b->sync(b->ctx,0)||b->reconstruction(b->ctx,&recon,&p->reconstruction)||b->sync(b->ctx,1)) return fail(p,"reconstruction failed");
    ps_fix_pdhg_collect_channels((const int32_t *)ps_frame_ptr(p,p->diagnostics.out_n_used),
        (const int32_t *)ps_frame_ptr(p,p->diagnostics.out_early_stopped),
        (const int32_t *)ps_frame_ptr(p,p->diagnostics.out_overflow_ch),
        (const uint32_t *)ps_frame_ptr(p,p->diagnostics.out_rho_q16),
        (const uint64_t *)ps_frame_ptr(p,p->diagnostics.out_res_sq),
        (const uint64_t *)ps_frame_ptr(p,p->diagnostics.out_d_sq),
        (const uint64_t *)ps_frame_ptr(p,p->diagnostics.out_mac_count),3,&p->channels);
    if(p->reconstruction.sat_events||p->reconstruction.mac_overflow||p->reconstruction.out_overflow||
       p->reconstruction.gain_clamped||p->channels.overflow_ch[0]||p->channels.overflow_ch[1]||p->channels.overflow_ch[2])
        return fail(p,"reconstruction numerical integrity failure");
    p->completed=1;return 0;
}

int ps_frame_write_result(const ps_frame_pipeline_t *p,const char *prefix) {
    char path[512];FILE *f;uint32_t z,x,c;uint64_t n;
    const int32_t *image;int32_t max=1;
    if(!p||!p->completed||!prefix||strlen(prefix)>450) return -1;
    n=ps_psm_hidden_len(&p->model);image=(const int32_t *)ps_frame_ptr(p,p->image);
    snprintf(path,sizeof(path),"%s.codes.bin",prefix);f=fopen(path,"wb");if(!f)return -1;
    if(fwrite(image,4,(size_t)(3*n),f)!=3*n){fclose(f);return -1;}if(fclose(f))return -1;
    for(c=0;c<3;++c)for(x=0;x<n;++x)if(image[c*n+x]>max)max=image[c*n+x];
    snprintf(path,sizeof(path),"%s.ppm",prefix);f=fopen(path,"wb");if(!f)return -1;
    fprintf(f,"P6\n%u %u\n255\n",p->model.hidden_w,p->model.hidden_h);
    for(z=0;z<p->model.hidden_h;++z)for(x=0;x<p->model.hidden_w;++x)for(c=0;c<3;++c){
        unsigned char v=(unsigned char)((uint64_t)(image[c*n+z*p->model.hidden_w+x]>0?image[c*n+z*p->model.hidden_w+x]:0)*255/max);
        if(fwrite(&v,1,1,f)!=1){fclose(f);return -1;}
    }
    if(fclose(f))return -1;
    snprintf(path,sizeof(path),"%s.json",prefix);f=fopen(path,"wb");if(!f)return -1;
    fprintf(f,"{\"geometry_hash\":%u,\"model_crc32\":%u,\"selected_id\":%u,\"margin_q16\":%d,\"low_confidence\":%u,\"frames\":%u,\"measurement_exp\":%d,\"image_scale_exp\":%d,\"lambda_kernel_q16\":%d,\"n_used\":[%d,%d,%d],\"rho_q16\":[%u,%u,%u],\"image_code_fraction_bits\":18,\"input_scale_num\":%llu,\"input_scale_den\":%llu,\"image_layout\":\"channel_major_row_major\",\"preview_gain\":%.12g}\n",
        p->model.geometry_hash,p->model.payload_crc32,p->selected_id,p->localization.margin_q16,p->localization.low_conf,p->frames,
        p->measurement.exponent,p->image_scale_exp,p->lambda_q16,p->channels.n_used[0],p->channels.n_used[1],p->channels.n_used[2],
        p->channels.rho_q16[0],p->channels.rho_q16[1],p->channels.rho_q16[2],
        (unsigned long long)p->input_scale_num,(unsigned long long)p->input_scale_den,255.0/max);
    {int bad=ferror(f);if(fclose(f))bad=1;return bad?-1:0;}
}
