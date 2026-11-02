#include "vision_grip.h"
#include <math.h>
#include <string.h>
bool vision_grip_begin(vision_grip_t *s,float accepted,float measured,float floor) {
    if(!s||!isfinite(accepted)||!isfinite(measured)||!isfinite(floor)||
       floor<0||accepted>4095||measured>4095||accepted<floor||measured<floor||
       fabsf(accepted-measured)>=80)return false;
    memset(s,0,sizeof(*s));s->state=VG_SEEK;s->target=accepted;s->floor=floor;
    s->last_measured=measured;s->have_measurement=true;return true;
}
void vision_grip_step(vision_grip_t *s,bool healthy,bool fresh,unsigned dt,
    float accepted,float measured) {
    if(s->state!=VG_SEEK&&s->state!=VG_PRESS&&s->state!=VG_READY)return;
    if(!isfinite(accepted)||!isfinite(measured)||measured<s->floor||measured>4095) {
        s->state=VG_FAILED;return;
    }
    if(dt>50) {
        /* 一次迟到不该判夹持失败（控制台回显/USB 背压/总线读超时都会发生），
         * 冻结本周期，连续 3 次才放弃。 */
        if(++s->late_cycles>=3)s->state=VG_FAILED;
        return;
    }
    s->late_cycles=0;
    if(!healthy) {
        s->bad_ms+=dt;s->stable_ms=0;
        if(s->bad_ms>500)s->state=VG_FAILED;
        return;
    }
    s->bad_ms=0;
    if(s->state==VG_READY)return;
    if((s->elapsed_ms+=dt)>12000){s->state=VG_FAILED;return;}
    s->read_ms+=dt;
    if(!fresh)return;
    unsigned read_ms=s->read_ms;s->read_ms=0;
    float lag=measured-accepted;
    bool stopped=fabsf(measured-s->last_measured)<=2;
    s->last_measured=measured;
    if(lag>80){s->state=VG_FAILED;return;}
    if(s->state==VG_SEEK) {
        if(lag>=30&&stopped)s->stable_ms+=read_ms;else s->stable_ms=0;
        if(s->stable_ms>=300) {
            s->contact=measured;
            s->target=fmaxf(s->floor,fminf(accepted,measured-50));
            s->press_ms=s->stable_ms=0;s->state=VG_PRESS;return;
        }
        if(lag<30&&fabsf(s->target-accepted)<=3)
            s->target=fmaxf(s->floor,accepted-9);
        if(accepted<=s->floor+3&&lag<20){s->state=VG_FAILED;return;}
    } else {
        s->press_ms+=read_ms;
        if(fabsf(accepted-s->target)<=3&&lag>=25&&lag<=80&&stopped)s->stable_ms+=read_ms;
        else s->stable_ms=0;
        if(s->stable_ms>=300){s->state=VG_READY;return;}
        /* A moving finger can briefly look obstructed. If it follows the
         * press target, continue seeking instead of claiming a firm hold. */
        if(s->press_ms>=600&&lag<25) {
            s->state=VG_SEEK;s->stable_ms=0;s->target=accepted;
        }
    }
}
