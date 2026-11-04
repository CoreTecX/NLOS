#ifndef PS_FRAME_PIPELINE_H
#define PS_FRAME_PIPELINE_H
#include "board_top.h"
#include "psm_format.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Inputs and outputs are arena-owned. Backend callbacks must execute real
 * arithmetic; a shadow register window is deliberately not a backend. */
typedef struct {
    void *ctx;
    int (*preprocess)(void *, uint64_t, uint32_t, uint64_t, uint32_t, uint32_t);
    int (*projection)(void *, const ps_fix_proj_job_t *, ps_fix_proj_result_t *);
    int (*reconstruction)(void *, const ps_fix_pdhg_job_t *, ps_fix_pdhg_result_t *);
    int (*sync)(void *, int cpu_access);
} ps_frame_backend_t;

typedef struct {
    ps_board_arena_t *arena;
    ps_psm_header_t model;
    uint64_t records, loc_u, gains, raw, diff, meas, scores, score_ch;
    uint64_t selected_u, selected_s, selected_v, image;
    ps_fix_pdhg_scratch_t scratch;
    ps_fix_pdhg_outputs_t diagnostics;
    uint32_t nx, ny, nz, frames, iterations;
    int32_t lambda_q16;
    int legacy_layout; /* PSM1 forward_loc: measurement/hidden vectors are F order. */
    int image_scale_exp;
    uint64_t input_scale_num,input_scale_den;
    uint32_t selected_id;
    ps_meas_norm_result_t measurement;
    ps_fix_proj_result_t localization;
    ps_fix_pdhg_result_t reconstruction;
    ps_fix_pdhg_channels_t channels;
    int initialized, completed;
    char error[256];
} ps_frame_pipeline_t;

int ps_frame_expand_diff24(void *buffer,size_t capacity_bytes,uint32_t samples);
int ps_frame_init(ps_frame_pipeline_t *, ps_board_arena_t *, const char *model,
                  uint32_t geometry_hash, uint32_t nx, uint32_t ny, uint32_t nz,
                  int legacy_layout);
int ps_frame_process(ps_frame_pipeline_t *, const char *const *frames, uint32_t count,
                     const ps_frame_backend_t *);
int ps_frame_write_result(const ps_frame_pipeline_t *, const char *prefix);
int ps_frame_publish_display(const ps_frame_pipeline_t *,volatile uint32_t *,uint32_t wall[9216]);
#if defined(__linux__)
int ps_frame_present_fbdev(const ps_frame_pipeline_t *,const uint32_t wall[9216],const char *,int);
#endif
void *ps_frame_ptr(const ps_frame_pipeline_t *, uint64_t phys);

#ifdef __cplusplus
}
#endif
#endif
