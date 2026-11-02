#include "vision_calibrate.h"
#include <math.h>
#include <string.h>
void vision_calibrate_start(vision_calibrate_t *s) {
    memset(s,0,sizeof(*s));s->active=true;
}
static bool fail_at(vision_calibrate_t *s,unsigned line) {s->active=false;s->failed=true;s->failure_line=line;return false;}
#define fail(s) fail_at(s,__LINE__)
bool vision_calibrate_step(vision_calibrate_t *s,const fpga_frame_t *f,
    bool fresh,bool healthy,bool permitted,unsigned dt,
    const float q[3],const float cmd[2],float out[2]) {
    out[0]=cmd[0];out[1]=cmd[1];
    if(!s->active)return false;
    if(!permitted)return fail(s);
    if(dt>50) {
        /* 一次迟到（控制台回显、USB 背压、总线读超时）不该毁掉整轮标定：
         * 冻结本周期，连续 3 次才判主循环真的卡死。 */
        if(++s->late_cycles>=3)return fail(s);
        return true;
    }
    s->late_cycles=0;
    s->elapsed_ms+=dt;
    if(s->elapsed_ms>90000)return fail(s);
    if(!healthy){s->count=0;return true;}
    if(!f||!f->visual_enabled||!f->found_red||!f->found_blue||!f->goal_valid)return fail(s);
    float r=hypotf((float)f->red_x-f->pole_x,(float)f->red_y-f->pole_y);
    if(!s->initialized) {
        memcpy(s->origin,q,sizeof(s->origin));memcpy(s->command,cmd,sizeof(s->command));
        s->pole_x=f->pole_x;s->pole_y=f->pole_y;s->target_x=f->blue_x;s->target_y=f->blue_y;
        s->baseline_radius=r;s->initialized=true;s->pause_ms=200;
    }
    /* 24 tick 的实测偏移在重力下垂 / 一次 12 tick 修正后就可能被顶破
     * （现场 25 tick 直接判失败，只差 1 tick）。放宽到 32 tick，仍远小于
     * 100 tick 跟随误差保护，也仍要求回位与端点位移独立。 */
    if(f->pole_x!=s->pole_x||f->pole_y!=s->pole_y||
       hypotf((float)f->blue_x-s->target_x,(float)f->blue_y-s->target_y)>5||
       (s->phase>0&&fabsf(q[0]-s->origin[0])>7)||fabsf(q[1]-s->origin[1])>32||fabsf(q[2]-s->origin[2])>32)
        return fail(s);
    if(s->pending) {
        unsigned axis=s->phase-1;
        s->pending_ms+=dt;
        if(s->pending_ms>3000)return fail(s);
        out[axis]=s->pending_target;
        if(fabsf(cmd[axis]-s->pending_target)<=.5f){s->pending=false;s->pause_ms=200;}
        return true;
    }
    if(s->pause_ms){s->pause_ms=s->pause_ms>dt?s->pause_ms-dt:0;return true;}
    if(!fresh)return true;
    float v[3]={q[1],q[2],r};
    if(!s->count) {
        for(int i=0;i<3;i++){s->sum[i]=0;s->minimum[i]=s->maximum[i]=v[i];}
        s->base_minimum=s->base_maximum=q[0];
    }
    s->base_minimum=fminf(s->base_minimum,q[0]);s->base_maximum=fmaxf(s->base_maximum,q[0]);
    for(int i=0;i<3;i++) {
        s->minimum[i]=fminf(s->minimum[i],v[i]);s->maximum[i]=fmaxf(s->maximum[i],v[i]);
        s->sum[i]+=v[i];
    }
    if(++s->count<5)return true;
    s->count=0;
    if(s->maximum[0]-s->minimum[0]>4||s->maximum[1]-s->minimum[1]>4||
       s->maximum[2]-s->minimum[2]>=10||s->base_maximum-s->base_minimum>4)return true;
    for(int i=0;i<3;i++)v[i]=s->sum[i]/5;
    /* Forward differences in two independent measured directions. Reversing
     * this loaded arm requires >100 ticks of following error; do not force a
     * return. Each axis moves only +16 actual ticks, inside the same 24/80
     * measured/command bounds, and every endpoint is observed after settling. */
    if(s->phase==0) {
        s->baseline_radius=v[2];s->origin[0]=q[0];s->origin[1]=v[0];s->origin[2]=v[1];
        memcpy(s->endpoints[0],v,sizeof(v));
        memcpy(s->command,cmd,sizeof(s->command));s->phase=1;
        return true;
    }
    unsigned axis=s->phase-1;
    float displacement=v[axis]-s->origin[axis+1];
    if(displacement<12) {
        /* Sample the actual endpoint in the bounded 12..32 tick region.
         * Never reverse to chase a nominal +16 endpoint after overshoot. */
        float delta=fminf(12,16-displacement);
        /* Bound the proposed command around the measured baseline. The
         * existing gravity hold bias can already be 90 ticks; counting its
         * removal as probe travel exhausts the budget before any joint moves. */
        if(++s->corrections>12||fabsf(cmd[axis]+delta-s->origin[axis+1])>240||
           fabsf(cmd[axis]+delta-q[axis+1])>=200)return fail(s);
        out[axis]+=delta;s->pending_target=out[axis];s->pending=true;s->pending_ms=0;return true;
    }
    memcpy(s->endpoints[s->phase],v,sizeof(v));
    s->corrections=0;s->pause_ms=200;
    if(++s->phase<=2)return true;
    float a=s->endpoints[1][0]-s->endpoints[0][0],b=s->endpoints[1][1]-s->endpoints[0][1];
    float c=s->endpoints[2][0]-s->endpoints[1][0],d=s->endpoints[2][1]-s->endpoints[1][1];
    float x=s->endpoints[1][2]-s->endpoints[0][2],y=s->endpoints[2][2]-s->endpoints[1][2];
    float determinant=a*d-b*c;
    s->measured_matrix[0]=a;s->measured_matrix[1]=b;
    s->measured_matrix[2]=c;s->measured_matrix[3]=d;
    s->radius_delta[0]=x;s->radius_delta[1]=y;
    /* A zero component is a valid derivative. Require observable combined
     * response and independent measured motion, not two nonzero components. */
    if(fabsf(determinant)<100||hypotf(x,y)<1)return fail(s);
    s->gradient[0]=(x*d-y*b)/determinant;s->gradient[1]=(a*y-c*x)/determinant;
    if(!isfinite(s->gradient[0])||!isfinite(s->gradient[1])||fabsf(s->gradient[0])>10||
       fabsf(s->gradient[1])>10||hypotf(s->gradient[0],s->gradient[1])<.02f)return fail(s);
    s->active=false;s->done=true;return true;
}
