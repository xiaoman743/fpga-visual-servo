#include "vision_probe.h"
#include <math.h>
bool vision_probe_arm(vision_probe_t *s,unsigned axis,float current,float delta) {
    if(!s||s->active||(axis!=1&&axis!=2)||!isfinite(current)||
       !isfinite(delta)||fabsf(delta)>20||fabsf(delta)<0.01f)return false;
    *s=(vision_probe_t){.active=true,.axis=axis,.target=current+delta};return true;
}
bool vision_probe_step(vision_probe_t *s,float dt_ms,bool permitted,bool healthy,
                       float accepted,float *proposal) {
    if(!s||!s->active||!proposal)return false;
    if(!permitted||!isfinite(dt_ms)||dt_ms<=0||dt_ms>50||!isfinite(accepted)) {
        s->active=false;return false;
    }
    s->elapsed_ms+=dt_ms;
    if(s->elapsed_ms>3000||fabsf(accepted-s->target)<0.01f) {
        s->active=false;return false;
    }
    if(!healthy)return false;
    *proposal=s->target;return true;
}
