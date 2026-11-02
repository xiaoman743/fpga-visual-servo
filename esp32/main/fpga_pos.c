#include "fpga_pos.h"
#include "vision_reach.h"
#include <math.h>
#include <string.h>
// Target/model/map state is owned by the main control task.
static bool active,armed;
static bool auto_enabled,auto_pan,auto_reach;
static int pending=-1;
static unsigned gains=100;
static float target[6],measured[6],bias[6];
static float off2=3043,off3=1011,height_sign=-1;
static bool height_ready;
static bool have_pole;
static int16_t last_pole_x,last_pole_y;
static vision_reach_map_t map;
static vision_reach_track_t track;
static unsigned online_count;
static float clamp(float x,float a,float b){return fmaxf(a,fminf(b,x));}
void fpga_pos_reset(void) {
    active=armed=have_pole=auto_enabled=auto_pan=auto_reach=false;__atomic_store_n(&pending,-1,__ATOMIC_RELEASE);
    memset(target,0,sizeof(target));
    vision_reach_clear(&map);
    memset(&track,0,sizeof(track));online_count=0;
}
void fpga_pos_service(void) {
    int request=__atomic_exchange_n(&pending,-1,__ATOMIC_ACQ_REL);
    if(request>=0){active=request!=0;armed=false;}
}
void fpga_pos_set_active(bool on){__atomic_store_n(&pending,on?1:0,__ATOMIC_RELEASE);}
bool fpga_pos_selected(void){return active;}
bool fpga_pos_active(void){return active&&armed;}
bool fpga_pos_needs_arm(void){return active&&!armed;}
void fpga_pos_note_armed(void){}
void fpga_pos_set_gain_pct(int p,int s,int e) {
    unsigned g=(unsigned)p|((unsigned)s<<8)|((unsigned)e<<16);
    __atomic_store_n(&gains,g,__ATOMIC_RELEASE);
}
bool fpga_pos_axis_owned(unsigned axis) {
    if(auto_enabled)return active&&armed&&axis<3&&(axis==0?auto_pan:auto_reach);
    unsigned g=__atomic_load_n(&gains,__ATOMIC_ACQUIRE);
    return active&&(axis==0 ? (g&255)!=0 : axis<3&&((g>>8)&65535)!=0);
}
void fpga_pos_arm_at_command(const int16_t meas[6],const float cmd[6]) {
    if(!meas||!cmd)return;
    for(int i=0;i<6;i++) {
        target[i]=clamp(cmd[i],FPGA_POS_MIN,FPGA_POS_MAX);
        measured[i]=(float)meas[i];bias[i]=target[i]-measured[i];
    }
    armed=true;
}
void fpga_pos_arm(const int16_t meas[6]) {
    float cmd[6];for(int i=0;i<6;i++)cmd[i]=meas[i];fpga_pos_arm_at_command(meas,cmd);
}
void fpga_pos_set_meas(const int16_t meas[6]){for(int i=0;i<6;i++)measured[i]=meas[i];}
static float axis_lo[2]={FPGA_POS_MIN,FPGA_POS_MIN};
static float axis_hi[2]={FPGA_POS_MAX,FPGA_POS_MAX};
void fpga_pos_set_limits(float lo2,float hi2,float lo3,float hi3) {
    if(isfinite(lo2)&&isfinite(hi2)&&hi2>lo2){axis_lo[0]=lo2;axis_hi[0]=hi2;}
    if(isfinite(lo3)&&isfinite(hi3)&&hi3>lo3){axis_lo[1]=lo3;axis_hi[1]=hi3;}
}
void fpga_pos_set_height_model(bool ready,float a,float b,float sign) {
    height_ready=ready;off2=a;off3=b;height_sign=sign;
}
void fpga_pos_commit(const float accepted[3]) {
    if(active&&armed)for(int i=0;i<3;i++)target[i]=accepted[i];
}
void fpga_pos_clear_samples(void){vision_reach_clear(&map);track.ready=false;online_count=0;}
bool fpga_pos_add_sample(float q1,float q2,float q3,float r2,float r3){return vision_reach_add(&map,q1,q2,q3,r2,r3);}
unsigned fpga_pos_sample_count(void){return map.count;}
void fpga_pos_observe_frame(const fpga_frame_t *f) {
    if(!f||!f->v2)return;
    if(have_pole&&(last_pole_x!=f->pole_x||last_pole_y!=f->pole_y))vision_reach_clear(&map);
    have_pole=true;last_pole_x=f->pole_x;last_pole_y=f->pole_y;
}
void fpga_pos_auto_gates(bool enabled,bool pan,bool reach){auto_enabled=enabled;auto_pan=pan;auto_reach=reach;if(!enabled||!reach)track.ready=false;}
unsigned fpga_pos_online_count(void){return online_count;}
void fpga_pos_track_frame(const fpga_frame_t *f) {
    if(!active||!armed||!auto_enabled||!auto_reach||!height_ready||!f||!f->v2||
       !f->visual_enabled||!f->found_red||!f->found_blue||!f->goal_valid)return;
    float radius=hypotf((float)f->red_x-f->pole_x,(float)f->red_y-f->pole_y);
    if(vision_reach_observe(&map,&track,measured[0],measured[1],measured[2],radius,off2,off3,height_sign))online_count++;
}
bool fpga_pos_map_ready(void) {
    float d2,d3;
    return height_ready&&vision_reach_solve(&map,measured[0],measured[1],measured[2],off2,off3,height_sign,1,&d2,&d3);
}
void fpga_pos_step_frame(const fpga_frame_t *f) {
    if(!active||!armed||!f||!f->found_red||!f->found_blue||!f->visual_enabled||!f->goal_valid)return;
    unsigned g=__atomic_load_n(&gains,__ATOMIC_ACQUIRE);
    float pan_gain=(float)(g&255)/100;
    unsigned sg=(g>>8)&255,eg=(g>>16)&255;
    float reach_gain=(float)(sg>eg?sg:eg)/100;
    if(auto_enabled){pan_gain=auto_pan?1:0;reach_gain=auto_reach?0.35f:0;}
    float candidate[3]={target[0],target[1],target[2]};
    if(auto_enabled) {
        float rx=(float)f->red_x-f->pole_x,ry=(float)f->red_y-f->pole_y;
        float gx=(float)f->goal_x-f->pole_x,gy=(float)f->goal_y-f->pole_y;
        float cross=rx*gy-ry*gx;
        float br=hypotf(gx,gy);
        float lateral=br>1.0f?fabsf(cross)/br:0.0f;
        /* 横向进入死区后不再纠底座：否则伸出后期每帧 ±2 tick 地蹭 CH1，
         * 看起来像“一直在纠正底座”，还会把已经对好的横向又推出去。 */
        if(lateral>FPGA_POS_PAN_DEADBAND_PX)
            candidate[0]-=clamp(cross*(4.0f/32768.0f)*FPGA_POS_TICK_TO_SERVO*pan_gain,-2,2);
    } else candidate[0]-=clamp(f->pan_inc*FPGA_POS_TICK_TO_SERVO*pan_gain,-8,8);
    if(reach_gain>0&&height_ready&&f->v2) {
        float rm=hypotf((float)f->red_x-f->pole_x,(float)f->red_y-f->pole_y);
        float rg=hypotf((float)f->goal_x-f->pole_x,(float)f->goal_y-f->pole_y);
        float err=rg-rm;
        float radial_limit=auto_enabled?4.0f:1.0f;
        float step=fabsf(err)<(auto_enabled?2.5f:4) ? 0:clamp(err*0.15f*reach_gain,-radial_limit,radial_limit);
        float d2,d3;
        if(vision_reach_solve_limited(&map,measured[0],measured[1],measured[2],off2,off3,height_sign,step,
                                      axis_lo[0],axis_hi[0],axis_lo[1],axis_hi[1],&d2,&d3)) {
            candidate[1]+=d2;candidate[2]+=d3;
        }
    }
    float scale=1;
    for(int i=0;i<3;i++) {
        float lo=fmaxf(FPGA_POS_MIN,measured[i]+bias[i]-FPGA_POS_LEAD_TICKS);
        float hi=fminf(FPGA_POS_MAX,measured[i]+bias[i]+FPGA_POS_LEAD_TICKS);
        float d=candidate[i]-target[i];
        if(d>0)scale=fminf(scale,fmaxf(0,(hi-target[i])/d));
        if(d<0)scale=fminf(scale,fmaxf(0,(lo-target[i])/d));
    }
    for(int i=0;i<3;i++)target[i]+=(candidate[i]-target[i])*scale;
}
bool fpga_pos_get(int16_t out[6]) {
    if(!out||!active||!armed)return false;
    for(int i=0;i<6;i++)out[i]=(int16_t)lroundf(target[i]);
    return true;
}
bool fpga_pos_get_float(float out[6]) {
    if(!out||!active||!armed)return false;
    memcpy(out,target,sizeof(target));return true;
}
