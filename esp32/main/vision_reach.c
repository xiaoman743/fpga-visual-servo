#include "vision_reach.h"
#include <math.h>
#include <string.h>
/* 高度模式，由主循环按实测高度在每周期喂进来（求解器自己不知道绝对高度）：
 *   0 = 等高切线（原行为，默认）
 *   1 = 允许"边伸边抬"（沿半径梯度，梯度方向确实在抬高时）
 *   2 = 已超高度上限，只许下降
 * 2026-11-02 现场：不给天花板时，求解器闭着眼往上抬，超限后 path_allowed
 * 每帧整步拒绝（20 s 内 402 次 [HEIGHT] rise blocked），形成
 * "想抬 → 已超限 → 不许抬 → 更下不来" 的死锁。 */
static int s_height_mode=0;
void vision_reach_set_height_mode(int mode){if(mode>=0&&mode<=2)s_height_mode=mode;}
int vision_reach_height_mode(void){return s_height_mode;}
void vision_reach_clear(vision_reach_map_t *m){memset(m,0,sizeof(*m));}
bool vision_reach_add(vision_reach_map_t *m,float q1,float q2,float q3,float dr2,float dr3) {
    if(!m||m->count>=VISION_REACH_SAMPLES||!isfinite(q1)||q1<0||q1>4095||!isfinite(q2)||!isfinite(q3)||
       !isfinite(dr2)||!isfinite(dr3)||q2<0||q2>4095||q3<0||q3>4095||
       fabsf(dr2)>10||fabsf(dr3)>10||hypotf(dr2,dr3)<0.01f)return false;
    m->samples[m->count++]=(vision_reach_sample_t){q1,q2,q3,dr2,dr3,true};return true;
}
bool vision_reach_gradient(const vision_reach_map_t *m,float q1,float q2,float q3,float *out2,float *out3) {
    if(!m||!out2||!out3||!isfinite(q1)||!isfinite(q2)||!isfinite(q3)||
       q1<0||q1>4095||q2<0||q2>4095||q3<0||q3>4095)return false;
    float sum=0,gr2=0,gr3=0;
    for(unsigned i=0;i<m->count;i++) {
        const vision_reach_sample_t *s=&m->samples[i];
        float d2=(q1-s->q1)*(q1-s->q1)+(q2-s->q2)*(q2-s->q2)+(q3-s->q3)*(q3-s->q3);
        if(s->used&&d2<=10000) {
            float w=1.0f/(d2+25);sum+=w;gr2+=w*s->dr2;gr3+=w*s->dr3;
        }
    }
    if(sum>0){*out2=gr2/sum;*out3=gr3/sum;return true;}
    /* 演示模式（2026-11-02）：邻域内没有样本时，退到最近样本外推，不再返回 false。
     * 返回 false 会让 vision_auto 在 REACH 里触发整轮“现场补采”，而补采是今晚
     * 两次 ABORT 的唯一来源（vision_calibrate.c:42 探针 3 s、:84 命令预算 80 tick）。
     * 外推只沿用最近样本的梯度方向，仍然受每帧 ±4 tick、领先 60 tick、
     * 关节固定区间与 EEPROM 余量约束，不会无限跑飞。 */
    {
        const vision_reach_sample_t *best=NULL;float bestd=0;
        for(unsigned i=0;i<m->count;i++) {
            const vision_reach_sample_t *s=&m->samples[i];
            if(!s->used)continue;
            float d2=(q1-s->q1)*(q1-s->q1)+(q2-s->q2)*(q2-s->q2)+(q3-s->q3)*(q3-s->q3);
            if(!best||d2<bestd){best=s;bestd=d2;}
        }
        if(!best)return false;
        *out2=best->dr2;*out3=best->dr3;return true;
    }
}
static float height(float q2,float q3,float off2,float off3,float sign) {
    const float k=6.28318530718f/4096;
    float a=(q2-off2)*k*sign,b=(q3-off3)*k*sign;
    return 115*sinf(a)+136*sinf(a+b);
}
bool vision_reach_observe(vision_reach_map_t *m,vision_reach_track_t *t,
    float q1,float q2,float q3,float radius,float off2,float off3,float sign) {
    if(!m||!t||!isfinite(radius)||radius<20||!isfinite(off2)||!isfinite(off3)||
       (sign!=1&&sign!=-1))return false;
    float g2,g3;
    if(!vision_reach_gradient(m,q1,q2,q3,&g2,&g3)){t->ready=false;return false;}
    float h=height(q2,q3,off2,off3,sign);
    if(!t->ready){*t=(vision_reach_track_t){true,q1,q2,q3,radius,h};return false;}
    float d2=q2-t->q2,d3=q3-t->q3,norm=d2*d2+d3*d3,dr=radius-t->radius;
    if(fabsf(q1-t->q1)>8||fabsf(h-t->height)>8||norm>6400) {
        *t=(vision_reach_track_t){true,q1,q2,q3,radius,h};return false;
    }
    if(norm<1225||fabsf(dr)<3)return false;
    float prior2,prior3;
    if(!vision_reach_gradient(m,t->q1,t->q2,t->q3,&prior2,&prior3))return false;
    float expected=prior2*d2+prior3*d3;
    bool ok=fabsf(expected)>=2&&dr*expected>0&&fabsf(dr/expected)>=0.4f&&fabsf(dr/expected)<=2;
    if(ok) {
        // A measured secant corrects the prior only along the observed motion.
        // The transverse component remains from the independently sampled map.
        float residual=(dr-expected)/norm;
        float next2=prior2+residual*d2,next3=prior3+residual*d3;
        ok=vision_reach_add(m,q1,q2,q3,next2,next3);
    }
    *t=(vision_reach_track_t){true,q1,q2,q3,radius,h};return ok;
}
bool vision_reach_solve(const vision_reach_map_t *m,float q1,float q2,float q3,
                        float off2,float off3,float sign,float dr_px,float *dq2,float *dq3) {
    if(!m||!dq2||!dq3||!isfinite(q1)||q1<0||q1>4095||!isfinite(q2)||!isfinite(q3)||!isfinite(off2)||!isfinite(off3)||
       !isfinite(dr_px)||q2<0||q2>4095||q3<0||q3>4095||(sign!=1&&sign!=-1))return false;
    float gr2,gr3;
    if(!vision_reach_gradient(m,q1,q2,q3,&gr2,&gr3))return false;
    const float k=6.28318530718f/4096;
    float a=(q2-off2)*k*sign,b=(q3-off3)*k*sign;
    float h2=sign*k*(115*cosf(a)+136*cosf(a+b));
    float h3=sign*k*136*cosf(a+b);
    float norm=hypotf(h2,h3);
    if(norm<0.01f)return false;
    /* 已超过高度上限：沿 −h 方向下降（dh = h·(−h/|h|) = −|h| < 0，必然降）。
     * 这一段不追径向，只负责把姿态拉回可行域，破掉上限处的死锁。 */
    if(s_height_mode==2) {
        float d=2.0f;
        *dq2=-h2/norm*d;*dq3=-h3/norm*d;return true;
    }
    /* 2026-11-02 现场：等高切线在 h2/h3 同号时，唯一解是两轴反向（CH2 增、CH3 减）。
     * 但执行侧两轴实际跟进比例并不相同，等高约束被打散 → 工具一路下沉直到压住桌面，
     * 随后 cmd 领先实测 65 tick、补采把 80 tick 命令预算耗尽 → ABORT（line=84）。
     * 本姿态下 h=(−0.291,−0.121) mm/tick、gr=(−0.11,−0.29) px/tick 同为负：
     * 减小两轴 tick 既抬高又伸长。若沿半径梯度方向本身就在抬高，就直接走梯度。 */
    float gn=hypotf(gr2,gr3);
    float rise=(gn>1e-6f)?(h2*gr2+h3*gr3)/gn:0.0f;
    bool lift_mode=(s_height_mode==1&&gn>=0.02f&&rise>0.05f);
    float n2,n3;
    if(lift_mode){n2=gr2/gn;n3=gr3/gn;}
    else{n2=h3/norm;n3=-h2/norm;}
    float sensitivity=gr2*n2+gr3*n3;
    if(fabsf(sensitivity)<0.02f)return false;
    // Damped inversion along the chosen direction, bounded jointly.
    float scale=dr_px*sensitivity/(sensitivity*sensitivity+0.02f*0.02f);
    float a2=n2*scale,a3=n3*scale;
    float maximum=fmaxf(fabsf(a2),fabsf(a3));
    if(maximum>4){a2*=4/maximum;a3*=4/maximum;}
    /* 抬臂通道限速：不加上限时沿梯度走约 25 mm/s，会直冲 75 mm 高度上限；
     * 一旦被 path_allowed 拦住就回到“cmd 领先实测”的老路。 */
    if(lift_mode) {
        float dh=fabsf(h2*a2+h3*a3);
        if(dh>0.30f){float f=0.30f/dh;a2*=f;a3*=f;}
    }
    *dq2=a2;*dq3=a3;return true;
}
bool vision_reach_solve_limited(const vision_reach_map_t *m,float q1,float q2,float q3,
                                float off2,float off3,float sign,float dr_px,
                                float lo2,float hi2,float lo3,float hi3,
                                float *dq2,float *dq3) {
    if(!vision_reach_solve(m,q1,q2,q3,off2,off3,sign,dr_px,dq2,dq3))return false;
    float gr2,gr3;
    if(!vision_reach_gradient(m,q1,q2,q3,&gr2,&gr3))return false;
    float a2=*dq2,a3=*dq3;
    /* 肩顶到边界：夹住它，剩下的径向需求按 CH3 自己的图像梯度换算 */
    if((a2>0&&q2+a2>hi2)||(a2<0&&q2+a2<lo2)) {
        float fit=(a2>0)?hi2-q2:lo2-q2;
        if(fabsf(fit)<fabsf(a2)) {
            float need=dr_px-gr2*fit;
            if(fabsf(gr3)>=0.02f) { a2=fit; a3=need/gr3; }
        }
    }
    /* 肘顶到边界：对称处理，把需求交回肩 */
    if((a3>0&&q3+a3>hi3)||(a3<0&&q3+a3<lo3)) {
        float fit=(a3>0)?hi3-q3:lo3-q3;
        if(fabsf(fit)<fabsf(a3)) {
            float need=dr_px-gr3*fit;
            if(fabsf(gr2)>=0.02f) { a3=fit; a2=need/gr2; }
        }
    }
    float maximum=fmaxf(fabsf(a2),fabsf(a3));
    if(maximum>4){a2*=4/maximum;a3*=4/maximum;}
    *dq2=a2;*dq3=a3;return true;
}
