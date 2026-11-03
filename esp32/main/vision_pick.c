#include "vision_pick.h"
#include <math.h>
bool vision_pick_arm(vision_pick_t *p,float degrees) {
    if(!p||!isfinite(degrees)||degrees<1||degrees>45||p->state==VP_SETTLE||p->state==VP_LIFT)return false;
    p->armed=true;p->degrees=degrees;p->state=VP_IDLE;p->late_cycles=0;return true;
}
bool vision_pick_resume(vision_pick_t *p,float degrees) {
    if(!vision_pick_arm(p,degrees))return false;
    p->was_closed=false;return true;
}
void vision_pick_cancel(vision_pick_t *p) {
    p->armed=false;if(p->state==VP_SETTLE||p->state==VP_LIFT)p->state=VP_ABORT;
}
static float h(float q2,float q3,float o2,float o3,float sign) {
    const float k=6.28318530718f/4096;
    float a=(q2-o2)*k*sign,b=(q3-o3)*k*sign;
    return 115*sinf(a)+136*sinf(a+b);
}
bool vision_pick_step(vision_pick_t *p,bool closed,bool start_ok,bool healthy,
                      float accepted,float measured,float q3,float off2,float off3,float sign,
                      unsigned dt_ms,float *proposal) {
    if(!p||!proposal)return false;
    bool edge=closed&&!p->was_closed;p->was_closed=closed;
    *proposal=accepted;
    if(p->armed&&edge) {
        p->armed=false;
        if(!start_ok||!healthy||!isfinite(accepted)||!isfinite(measured)||!isfinite(q3)||!isfinite(off2)||!isfinite(off3)||
           (sign!=1&&sign!=-1)){p->state=VP_ABORT;return false;}
        float derivative=h(accepted+1,q3,off2,off3,sign)-h(accepted-1,q3,off2,off3,sign);
        if(fabsf(derivative)<0.01f){p->state=VP_ABORT;return false;}
        p->start=accepted;p->target=accepted+(derivative>0?1:-1)*p->degrees*4096/360;
        float previous=h(accepted,q3,off2,off3,sign);
        for(int i=1;i<=16;i++) {
            float q=accepted+(p->target-accepted)*(float)i/16;
            float next=h(q,q3,off2,off3,sign);
            if(q<0||q>4095||next<previous){p->state=VP_ABORT;return false;}
            previous=next;
        }
        p->measured_start=measured;p->height_start=h(measured,q3,off2,off3,sign);
        p->elapsed_ms=p->blocked_ms=p->verified_ms=p->unhealthy_ms=0;p->last_accepted=accepted;p->state=VP_SETTLE;
    }
    if(p->state!=VP_SETTLE&&p->state!=VP_LIFT)return false;
    if(!closed||!isfinite(measured)||!isfinite(q3)){p->state=VP_ABORT;return false;}
    if(dt_ms>50) {
        /* 抬升途中一次迟到只冻结目标，连续 3 次才判卡死。 */
        if(++p->late_cycles>=3){p->state=VP_ABORT;return false;}
        return true;
    }
    p->late_cycles=0;
    if(!healthy) {
        p->verified_ms=0;p->unhealthy_ms+=dt_ms;
        if(p->unhealthy_ms>500){p->state=VP_ABORT;return false;}
        return true; // Frozen proposal=accepted; no progress on rejected data.
    }
    p->unhealthy_ms=0;
    if(fabsf(accepted-measured)>=160){p->state=VP_ABORT;return false;}
    p->elapsed_ms+=dt_ms;
    if(p->state==VP_SETTLE){if(p->elapsed_ms>=800){p->state=VP_LIFT;p->elapsed_ms=0;}return true;}
    if(p->elapsed_ms>18000){p->state=VP_ABORT;return false;}
    if(fabsf(p->target-accepted)<=0.5f) {
        /* Operator-selected command completion. DONE proves command arrival,
         * not physical travel or an object leaving the table. Runtime guards
         * above remain active throughout the commanded trajectory. */
        p->state=VP_DONE;
        return true;
    }
    if(fabsf(accepted-p->last_accepted)<0.01f)p->blocked_ms+=dt_ms;else p->blocked_ms=0;
    p->last_accepted=accepted;
    if(p->blocked_ms>1000){p->state=VP_ABORT;return false;}
    float step=80*(float)dt_ms/1000,remaining=p->target-accepted;
    *proposal=accepted+fmaxf(-step,fminf(step,remaining));return true;
}
