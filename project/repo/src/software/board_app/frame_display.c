#include "frame_pipeline.h"
#include "ps_pl_regs.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t image_rgb(const int32_t *im,uint32_t i,int32_t max) {
    uint32_t c,rgb=0;
    for(c=0;c<3;++c) {
        int32_t v=im[c*432+i];
        uint32_t code=v>0?(uint32_t)((uint64_t)v*255/max):0;
        rgb=(rgb<<8)|code;
    }
    return rgb;
}

int ps_frame_publish_display(const ps_frame_pipeline_t *p,volatile uint32_t *regs,
                             uint32_t wall[9216]) {
    const uint16_t *raw;
    const int32_t *im;
    const uint8_t *rec;
    uint32_t y,x,c,i,max_wall=1;
    int32_t max_image=1;
    if(!p||!p->completed||!regs||!wall)return -1;
    raw=(const uint16_t *)ps_frame_ptr(p,p->raw);
    im=(const int32_t *)ps_frame_ptr(p,p->image);
    rec=(const uint8_t *)ps_frame_ptr(p,p->records)+
        (uint64_t)p->selected_id*ps_psm_record_bytes_hdr(&p->model);
    /* Representative last exposure; reconstruction still uses all 1/4/8 frames.
     * Round the accumulated two greens exactly as the RAW preprocessor does. */
    for(y=0;y<96;++y)for(x=0;x<96;++x) {
        uint32_t sum[3]={0,0,0},dy,dx;
        for(dy=0;dy<20;++dy)for(dx=0;dx<20;++dx) {
            c=((dy&1)==(dx&1))?((dy&1)?2:0):1;
            sum[c]+=raw[(y*20+dy)*1920+x*20+dx];
        }
        {uint32_t g=sum[1]/2; if((sum[1]&1)&&(g&1))++g;sum[1]=g;}
        for(c=0;c<3;++c)if(sum[c]>max_wall)max_wall=sum[c];
        /* All values <=409500; temporary sums cannot fit in RGB888. */
    }
    for(y=0;y<96;++y)for(x=0;x<96;++x) {
        uint32_t sum[3]={0,0,0},dy,dx,rgb=0;
        for(dy=0;dy<20;++dy)for(dx=0;dx<20;++dx) {
            c=((dy&1)==(dx&1))?((dy&1)?2:0):1;
            sum[c]+=raw[(y*20+dy)*1920+x*20+dx];
        }
        {uint32_t g=sum[1]/2;if((sum[1]&1)&&(g&1))++g;sum[1]=g;}
        for(c=0;c<3;++c)rgb=(rgb<<8)|(sum[c]*255/max_wall);
        i=y*96+x;wall[i]=rgb;ps_pl_wall_pixel(regs,i,rgb);
    }
    for(i=0;i<1296;++i)if(im[i]>max_image)max_image=im[i];
    for(y=0;y<18;++y)for(x=0;x<24;++x) {
        uint32_t src=p->legacy_layout?x*18+y:y*24+x;
        uint8_t mask=ps_psm_read_mask_n(rec,&p->model,src);
        i=y*24+x;ps_pl_mask_entry(regs,i,mask);
        ps_pl_rec_pixel(regs,i,mask?image_rgb(im,i,max_image):0x808080u);
    }
    for(i=0;i<256;++i)ps_pl_osd_char(regs,i,' ');
    {char message[96];
        snprintf(message,sizeof(message),"ID=%u FRAMES=%u ITER=%u",p->selected_id,p->frames,p->iterations);
        for(i=0;message[i];++i)ps_pl_osd_char(regs,i,(uint8_t)message[i]);
    }
    return 0;
}

#if defined(__linux__)
#include <linux/fb.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

/* PS DP framebuffer fallback. No libdrm dependency. A missing fbdev fails
 * explicitly; DRM-only systems must enable fbdev emulation or use DRM tooling. */

/* Scaled 1920x1080 logical layout, honoring real stride, offsets, and RGB565
 * or RGB888 channel bitfields. force_gray=2 is a visibly marked diagnostic
 * preview and never alters PL confidence or validity decisions. */
static uint32_t fb_component(uint32_t v,struct fb_bitfield b) {
    return b.length ? ((v*((1u<<b.length)-1u)+127u)/255u)<<b.offset : 0;
}
static const uint8_t *glyph(char c) {
    static const char keys[]=" ACDEGILMNOPRSTUVW-:";
    static const uint8_t rows[][7]={
     {0,0,0,0,0,0,0},{14,17,17,31,17,17,17},{14,17,16,16,16,17,14},{30,17,17,17,17,17,30},
     {31,16,16,30,16,16,31},{14,17,16,23,17,17,15},{31,4,4,4,4,4,31},
     {16,16,16,16,16,16,31},{17,27,21,21,17,17,17},{17,25,25,21,19,19,17},
     {14,17,17,17,17,17,14},{30,17,17,30,16,16,16},{30,17,17,30,20,18,17},
     {15,16,16,14,1,1,30},{31,4,4,4,4,4,4},{17,17,17,17,17,17,14},
     {17,17,17,17,17,10,4},{17,17,17,21,21,21,10},{0,0,0,31,0,0,0},
     {0,4,4,0,4,4,0}};
    const char *q=strchr(keys,c);return rows[q?(size_t)(q-keys):0];
}
static int text_pixel(const char *text,uint32_t x,uint32_t y,uint32_t left,uint32_t top) {
    uint32_t k,dx,dy;if(x<left||y<top)return 0;
    dx=(x-left)/3;dy=(y-top)/3;k=dx/6;
    if(dy>=7||k>=strlen(text)||dx%6>=5)return 0;
    return (glyph(text[k])[dy]>>(4-dx%6))&1;
}
int ps_frame_present_fbdev(const ps_frame_pipeline_t *p,const uint32_t wall[9216],
                           const char *device,int force_gray) {
    struct fb_fix_screeninfo fix;struct fb_var_screeninfo var;
    uint8_t *map;int fd;uint32_t x,y,i,bytes,w,h,left,top;int32_t max=1;
    int gray=force_gray==1,diagnostic=force_gray==2;
    const int32_t *im=NULL;const uint8_t *rec=NULL;
    if(!device||(!gray&&(!p||!p->completed||!wall)))return -1;
    if(!gray) {
        im=(const int32_t *)ps_frame_ptr(p,p->image);
        rec=(const uint8_t *)ps_frame_ptr(p,p->records)+
            (uint64_t)p->selected_id*ps_psm_record_bytes_hdr(&p->model);
    }
    fd=open(device,O_RDWR|O_CLOEXEC);if(fd<0)return -1;
    if(ioctl(fd,FBIOGET_FSCREENINFO,&fix)||ioctl(fd,FBIOGET_VSCREENINFO,&var)||
       fix.type!=FB_TYPE_PACKED_PIXELS||fix.visual!=FB_VISUAL_TRUECOLOR||
       var.xres<320||var.yres<200||var.xres>8192||var.yres>8192||
       (var.bits_per_pixel!=16&&var.bits_per_pixel!=24&&var.bits_per_pixel!=32)||
       !var.red.length||!var.green.length||!var.blue.length||
       var.red.length>8||var.green.length>8||var.blue.length>8||var.transp.length>8||
       var.red.offset+var.red.length>var.bits_per_pixel||
       var.green.offset+var.green.length>var.bits_per_pixel||
       var.blue.offset+var.blue.length>var.bits_per_pixel||
       var.transp.offset+var.transp.length>var.bits_per_pixel||
       var.red.msb_right||var.green.msb_right||var.blue.msb_right||var.transp.msb_right||
       (uint64_t)var.xoffset+var.xres>var.xres_virtual||
       (uint64_t)var.yoffset+var.yres>var.yres_virtual||
       ((uint64_t)var.xoffset+var.xres)*(var.bits_per_pixel/8)>fix.line_length||
       ((uint64_t)var.yoffset+var.yres)*fix.line_length>fix.smem_len) {close(fd);return -1;}
    {uint32_t r=fb_component(255,var.red),g=fb_component(255,var.green),b=fb_component(255,var.blue),a=fb_component(255,var.transp);
     if((r&g)||(r&b)||(g&b)||((r|g|b)&a)){close(fd);return -1;}}
    bytes=var.bits_per_pixel/8;
    map=(uint8_t *)mmap(NULL,fix.smem_len,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
    if(map==MAP_FAILED){close(fd);return -1;}
    if(!gray)for(i=0;i<1296;++i)if(im[i]>max)max=im[i];
    w=var.xres;h=(uint32_t)((uint64_t)w*1080/1920);
    if(h>var.yres){h=var.yres;w=(uint32_t)((uint64_t)h*1920/1080);}
    left=(var.xres-w)/2;top=(var.yres-h)/2;
    for(y=0;y<var.yres;++y) {
        uint8_t *row=map+(uint64_t)(y+var.yoffset)*fix.line_length+(uint64_t)var.xoffset*bytes;
        for(x=0;x<var.xres;++x) {
            uint32_t rgb=gray?0x808080u:0x202020u;
            if(!gray&&x>=left&&x<left+w&&y>=top&&y<top+h) {
                uint32_t lx=(x-left)*1920/w,ly=(y-top)*1080/h;
                if(lx>=240&&lx<720&&ly>=120&&ly<600)rgb=wall[((ly-120)/5)*96+(lx-240)/5];
                if(lx>=1080&&lx<1800&&ly>=120&&ly<660) {
                    uint32_t iy=(ly-120)/30,ix=(lx-1080)/30;
                    uint32_t src=p->legacy_layout?ix*18+iy:iy*24+ix;
                    rgb=ps_psm_read_mask_n(rec,&p->model,src)?image_rgb(im,iy*24+ix,max):0x808080u;
                }
                if(diagnostic&&ly<80)rgb=0x902020;
                if(diagnostic&&text_pixel("DIAGNOSTIC PREVIEW - UNVALIDATED",lx,ly,180,25))rgb=0xffffff;
                if(text_pixel("MEASUREMENT",lx,ly,240,85)||text_pixel("RECONSTRUCTION",lx,ly,1080,85))rgb=0xffffff;
            }
            {uint32_t pixel=fb_component((rgb>>16)&255,var.red)|fb_component((rgb>>8)&255,var.green)|
                fb_component(rgb&255,var.blue)|fb_component(255,var.transp);
             memcpy(row+(uint64_t)x*bytes,&pixel,bytes);}
        }
    }
    __sync_synchronize();
    if(ioctl(fd,FBIOBLANK,FB_BLANK_UNBLANK)) {munmap(map,fix.smem_len);close(fd);return -1;}
    printf("FB_PRESENT=%ux%u BPP=%u STRIDE=%u DIAGNOSTIC=%d\n",var.xres,var.yres,var.bits_per_pixel,fix.line_length,diagnostic);
    munmap(map,fix.smem_len);close(fd);return 0;
}
#endif
