#include "boot_profile.h"
#include <math.h>
int boot_profile_margin_step(const float cmd[3],const float measured[3],
    const float lo[3],const float hi[3],float step,float next[3]) {
    if(!isfinite(step)||step<=0||step>1)return -1;
    bool recover=false;
    for(int i=0;i<3;i++) {
        next[i]=cmd[i];
        if(!isfinite(cmd[i])||!isfinite(measured[i])||!isfinite(lo[i])||!isfinite(hi[i])||
           hi[i]-lo[i]<16||cmd[i]<lo[i]-40||cmd[i]>hi[i]+40||
           measured[i]<lo[i]-40||measured[i]>hi[i]+40||fabsf(cmd[i]-measured[i])>=60)return -1;
    }
    for(int i=0;i<3;i++) {
        float goal=cmd[i];
        if(cmd[i]>hi[i]||measured[i]>hi[i])goal=hi[i]-8;
        else if(cmd[i]<lo[i]||measured[i]<lo[i])goal=lo[i]+8;
        else continue;
        recover=true;
        next[i]=cmd[i]+fmaxf(-step,fminf(step,goal-cmd[i]));
    }
    return recover?1:0;
}
bool boot_profile_restore(rt_level_t *l,rt_height_t *h,
    const float cmd[3],const float measured[2],const float lo[3],const float hi[3]) {
    /* Provenance: reference51-and-calibration.log, physically horizontal:
     * measured CH2/3/4 = 2391/2556/1178, commands = 2344/2537/1178;
     * operator measured tool height = 51 mm. Never capture the boot pose as level. */
    if(l->enabled||h->enabled)return false;
    rt_level_t next=*l;rt_height_t ht=*h;
    if(!rt_level_enable(&next,cmd,lo,hi)||!rt_level_seed_measurement(&next,measured))return false;
    next.ref[0]=2344;next.ref[1]=2537;next.ref[2]=1178;
    next.measured_ref[0]=2391;next.measured_ref[1]=2556;
    float ideal=1178-(measured[0]-2391)-(measured[1]-2556);
    float allowance=next.tolerance_deg*LEVEL_TICKS_PER_DEG;
    if(!isfinite(ideal)||ideal<lo[2]-allowance||ideal>hi[2]+allowance)return false;
    if(!rt_height_set_model(&ht,3043,1011,-1)||
       !rt_height_reference(&ht,(float[]){2391,2556},51)||
       !rt_height_enable(&ht,cmd,measured))return false;
    *l=next;*h=ht;return true;
}
