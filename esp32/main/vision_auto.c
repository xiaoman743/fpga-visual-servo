#include "vision_auto.h"
#include <math.h>
#include <string.h>
bool vision_auto_running(const vision_auto_t *s){return s&&s->state>=VA_WAIT&&s->state<=VA_LIFT;}
const char *vision_auto_name(vision_auto_state_t state) {
    static const char *names[]={"IDLE","WAIT_KEY3","ALIGN","REACH","SETTLE","CLOSE","LIFT","DONE","ABORT"};
    return state<=VA_ABORT?names[state]:"UNKNOWN";
}
bool vision_auto_arm(vision_auto_t *s,float degrees) {
    if(!s||vision_auto_running(s)||!isfinite(degrees)||degrees<1||degrees>45)return false;
    memset(s,0,sizeof(*s));s->state=VA_WAIT;s->lift_degrees=degrees;s->reason="waiting for KEY3";return true;
}
bool vision_auto_finish(vision_auto_t *s,float degrees) {
    if(!vision_auto_arm(s,degrees))return false;
    s->confirmed_endpoint=true;return true;
}
bool vision_auto_resume_lift(vision_auto_t *s,float degrees,const fpga_frame_t *f) {
    if(!s||!f||!f->v2||!f->visual_enabled||vision_auto_running(s)||
        !isfinite(degrees)||degrees<1||degrees>45)return false;
    memset(s,0,sizeof(*s));s->lift_degrees=degrees;
    s->state=VA_LIFT;s->have_config=s->close_issued=true;
    s->pole_x=f->pole_x;s->pole_y=f->pole_y;s->target_x=f->blue_x;s->target_y=f->blue_y;
    s->reason="explicit resume from verified existing hold";return true;
}
void vision_auto_cancel(vision_auto_t *s,const char *reason) {
    if(!s)return;
    if(s->state!=VA_IDLE){s->state=VA_ABORT;s->reason=reason;}
}
static void transition(vision_auto_t *s,vision_auto_state_t next) {
    s->state=next;s->elapsed_ms=s->stable_ms=s->stable_frames=0;s->late_cycles=0;s->stall_ms=0;s->reason="running";
}
vision_auto_output_t vision_auto_step(vision_auto_t *s,const vision_auto_input_t *in) {
    vision_auto_output_t out={0};
    if(!s||!in||!vision_auto_running(s))return out;
    out.freeze=true;
    if(in->manual_override){vision_auto_cancel(s,"manual override");return out;}
    if(!in->configured||!in->selected||!in->constraints_ready){vision_auto_cancel(s,"configuration/source/constraints lost");return out;}
    if(in->dt_ms>50){
        /* One late cycle is usually console/USB backpressure or a servo-bus
         * read timeout, not a lost approach. Freeze this cycle without
         * moving; only give up after three consecutive late cycles.
         * WAIT never aborts, so a status dump cannot disarm the button. */
        if(s->state!=VA_WAIT&&++s->late_cycles>=3)vision_auto_cancel(s,"control loop delayed");
        return out;
    }
    s->late_cycles=0;
    s->elapsed_ms+=in->dt_ms;
    if(!in->healthy) {
        s->unhealthy_ms+=in->dt_ms;
        s->stable_ms=s->stable_frames=0;
        if(s->unhealthy_ms>500)vision_auto_cancel(s,"feedback unhealthy >500ms");
        return out;
    }
    s->unhealthy_ms=0;
    if(!in->frame_fresh||!in->frame||!in->frame->v2) {
        s->stable_ms=s->stable_frames=0;
        s->vision_stale_ms+=in->dt_ms;
        if(s->state!=VA_WAIT&&s->vision_stale_ms>500)vision_auto_cancel(s,"vision frame stale >500ms");
        return out;
    }
    s->vision_stale_ms=0;
    const fpga_frame_t *f=in->frame;
    if(s->state==VA_WAIT) {
        if(!f->visual_enabled){s->key3_was_off=true;return out;}
        if(!s->key3_was_off)return out; // Require a fresh physical OFF -> ON after arming.
        if(!in->jaws_open)return out; // Main loop opens to the historic 1978-tick aperture.
        if(!f->found_red||!f->found_blue||!f->goal_valid)return out;
        s->pole_x=f->pole_x;s->pole_y=f->pole_y;s->target_x=f->goal_x;s->target_y=f->goal_y;
        s->radial_offset=in->expected_offset;s->have_config=true;
        transition(s,s->confirmed_endpoint?VA_SETTLE:VA_ALIGN);
    }
    if(s->elapsed_ms>240000&&s->state<=VA_SETTLE){vision_auto_cancel(s,"approach timeout");return out;}
    if(f->pole_x!=s->pole_x||f->pole_y!=s->pole_y||fabsf(in->expected_offset-s->radial_offset)>0.01f) {
        vision_auto_cancel(s,"pole or tool offset changed");return out;
    }
    if(!f->visual_enabled){vision_auto_cancel(s,"KEY3 stopped");return out;}
    // After close the object/marker may be occluded; never resume visual motion.
    if(s->state<=VA_SETTLE) {
        if(!f->found_red||!f->found_blue||!f->goal_valid){vision_auto_cancel(s,"target or marker lost");return out;}
        /* 用 FPGA 锁定/迟滞后的目标点 goal 判断“目标真的被挪走”，不要用原始
         * 质心 blue：夹爪靠近会遮住蓝块，blue 会跳 >18 px，而 goal 由
         * target_gate 稳稳锁住（连续 4 帧超 24 px 才重捕获）。 */
        /* 2026-11-03 现场：判据原本是 >18 px 连续 3 帧。臂伸到物体正上方时夹爪会
         * 遮住蓝块，质心朝可见部分跳 → target_gate 重捕获到偏移位置 → 误判"目标被
         * 挪走"（径向还差 20 px 时 ABORT）。真挪动物体时 goal 会跳 100+ px 且不会
         * 回来自我一致，所以把阈值放到 50 px、帧数放到 5 帧，只拦真移动。 */
        if(hypotf((float)f->goal_x-s->target_x,(float)f->goal_y-s->target_y)>50) {
            s->stable_ms=s->stable_frames=0;
            if(in->new_frame&&++s->shifted_frames>=5)vision_auto_cancel(s,"target moved (5 frames)");
            return out; // Freeze on the first uncertain frame, without motion.
        }
        s->shifted_frames=0;
    }
    if(in->calibrating)return out; // Keep all safety checks above, freeze visual approach.
    /* 领窗静默死锁检测（2026-11-03 现场）：fpga_pos 的抗饱和窗口会把目标钉在
     * measured±LEAD 的边界上。一旦关节没跟上（供电欠压、舵机丢包、机械卡滞都
     * 可能），目标就永久停在边界、既不动也不报错——现场表现为"运动到一半卡住"，
     * 而跟随差恰好恒等于 LEAD。这里把它变成一条明确的 ABORT，便于区分
     * "真的卡住" 与 "只是走得慢"。判据：实测几乎不动 且 跟随差已接近窗口上界，
     * 连续 1 秒即判定。 */
    if(s->state==VA_REACH||s->state==VA_SETTLE) {
        if(in->measured_motion<=1.0f&&in->position_error>=120.0f) s->stall_ms+=in->dt_ms;
        else s->stall_ms=0;
        if(s->stall_ms>1000) {vision_auto_cancel(s,"leader saturated: target at window edge, joint not following");return out;}
    }
    float rx=(float)f->red_x-f->pole_x,ry=(float)f->red_y-f->pole_y;
    float bx=(float)f->goal_x-f->pole_x,by=(float)f->goal_y-f->pole_y;
    float br=hypotf(bx,by),rr=hypotf(rx,ry);
    if(br<20||rr<20){vision_auto_cancel(s,"pole singularity");return out;}
    float lateral=fabsf(rx*by-ry*bx)/br;
    float radial=br+s->radial_offset-rr;
    /* 底座对准容差：现场 lateral 在十几 px 时仍在反复纠 CH1，4/8 px 都攒不满
     * 12 帧。放宽到 15 px；>22 px 才退回 ALIGN（5 px 迟滞，避免边界抖动）。 */
    bool aligned=lateral<=15&&rx*bx+ry*by>0;
    if(s->state==VA_ALIGN) {
        out.pan=true;out.freeze=false;
        if(in->new_frame) {if(aligned)s->stable_frames++;else s->stable_frames=0;}
        if(s->stable_frames>=12)transition(s,VA_REACH);
    } else if(s->state==VA_REACH) {
        if(fabsf(radial)<80&&aligned){transition(s,VA_SETTLE);return out;}
        if(!in->map_ready){out.calibrate=aligned;out.pan=!aligned;out.freeze=aligned;return out;}
        out.pan=true;out.reach=aligned;out.freeze=false;
        if(!aligned&&lateral>22){transition(s,VA_ALIGN);out.reach=false;}
    } else if(s->state==VA_SETTLE) {
        if(!s->confirmed_endpoint&&(fabsf(radial)>=80||!aligned)){transition(s,VA_REACH);return out;}
        if(in->new_frame&&in->measured_motion<=2&&in->joint_speed<=2) {s->stable_frames++;}
        else if(in->new_frame)s->stable_frames=0;
        s->stable_ms+=in->dt_ms;
        if(s->stable_ms>=800&&s->stable_frames>=12&&in->position_error<60) {
            if(!in->jaws_open){vision_auto_cancel(s,"gripper changed before close");return out;}
            out.arm_lift=out.close=true;s->close_issued=true;transition(s,VA_CLOSE);
        }
    } else if(s->state==VA_CLOSE) {
        if(s->elapsed_ms>13000){vision_auto_cancel(s,"gripper close timeout");return out;}
        if(in->lift_aborted){vision_auto_cancel(s,"lift preparation aborted");return out;}
        if(in->gripper_settled)transition(s,VA_LIFT);
    } else if(s->state==VA_LIFT) {
        if(in->lift_aborted){vision_auto_cancel(s,"lift aborted");return out;}
        if(in->lift_done){transition(s,VA_DONE);s->reason="cycle done; object contact unverified";}
        else if(s->elapsed_ms>20000)vision_auto_cancel(s,"lift timeout");
    }
    return out;
}
