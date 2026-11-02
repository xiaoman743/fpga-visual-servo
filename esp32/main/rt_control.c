/*
 * rt_control.c -- 接收端控制律：**增量式**位置控制（2026-10-02 晚定案）
 * ============================================================================
 * 架构说明见 rt_control.h 顶部。这里只强调三件事：
 *
 * ① 累加基准是指令位置 cmd_pos，不是实测位置 meas。
 *    增量式的经典坑：拿实测当基准时，舵机还没走到位，下一帧又基于旧位置加一次
 *    → 误差累积 → 飞车。用 cmd_pos 就与舵机延迟无关。
 *
 * ② 撞机械限位会自动停止 —— 这是本架构最重要的好处，也是操作者提出它的理由：
 *    顶住之后实测位置不再增加，但**我们的 cmd_pos 还在涨**（因为它不受限），
 *    于是 cmd_pos 与实测越差越大 → 被堵转保护判为卡住 → 松力矩。
 *    换句话说：增量式把"限位"和"卡住"统一成同一件事去处理，
 *    **不需要事先知道每个轴的行程**。今天一半的 bug 都出在"猜行程"上。
 *
 * ③ ★★ 2026-10-02 深夜第三轮实测后：CH1/CH2/CH5 改为**速率式** ★★
 *    增量位置式在实测里被操作者判为"不符合使用逻辑"：推到底是**一次性**累加
 *    固定位移（还被 RC_DW_MAX_TICKS 钳掉大半），松手原路退回，想持续转只能泵。
 *    速率式 = 通道值直接当速度指令：**推住 = 匀速转、推得越深越快、松手 = 立刻停**。
 *    rate 由 rc_apply_ctrl 更新，rc_step 按 dt 积分到 cmd_pos（累加基准仍是 cmd_pos）。
 *    防撞限位：反缠绕（anti-windup）—— cmd_pos 与实测差超过 RC_WINDUP_TICKS 时
 *    每帧往回收 RC_WINDUP_PULL，误差停在 200（> RC_STALL_TICKS=180）→ 堵转保护仍触发。
 *    CH3/CH4（手腕姿态镜像）保持原增量式，操作者确认手感没问题。
 *    CH6 夹爪后来也离开了增量式 → **双态绝对**（修"合不拢、夹不紧"），
 *    根因与纪律见 rt_control.h 的 RC_GRIP_* 注释。
 * ============================================================================
 */
#include "rt_control.h"

#include <math.h>
#include <string.h>

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void rc_sync_to_meas(rc_joint_t *j)
{
    for (int i = 0; i < RC_NJ; i++) {
        j[i].cmd_pos = (float)j[i].meas;
        j[i].vel = 0.0f;
        j[i].rate = 0.0f;
        j[i].carry = 0.0f;
    }
}
bool rc_stall_step(rc_stall_guard_t *s,bool monitor,int64_t now,float command,
    int16_t measured,float error_limit,unsigned timeout_ms,int movement_limit) {
    if(!monitor||!isfinite(command)||fabsf(command-measured)<=error_limit) {
        s->since_us=0;return false;
    }
    if(!s->since_us){s->since_us=now;s->reference=measured;return false;}
    return now-s->since_us>(int64_t)timeout_ms*1000&&
        fabsf((float)measured-s->reference)<movement_limit;
}

/* ★ 哪些通道走速率式：CH1 底座(yaw)、CH2 大臂(摇杆Y)、CH5 腕旋转(摇杆X)、
 * CH3 小臂(pitch)、CH4 腕俯仰(roll)。
 * ★★ 2026-10-03 操作者实测后 CH3/CH4 也转速率式：姿态镜像是"手腕回正 = 关节
 *    跟着回退"，操作者明确不要回转（"转完 mpu 又回转了，我不需要回转"）。
 *    速率式语义 = 压住持续动、回正停在原地，与 CH1/2/5 手感统一。
 *    通道值仍是姿态角（遥控端不用改），只是接收端把它当**速度**解释。
 * CH6 夹爪保持双态绝对（它要的就是两个确定位置）。 */
static const bool RC_RATE_CH[RC_NJ] = {true, true, true, true, true, false};

/* ---- CH6 夹爪：双态绝对（根因与纪律见 rt_control.h 的 RC_GRIP_* 注释）---- */
static float s_grip_open;   /* 张开目标 tick（= EEPROM limit_max） */
static float s_grip_close;  /* 闭合目标 tick（= limit_min − RC_GRIP_PRESS，故意越过） */
static bool s_grip_cfg;     /* 限位读到了才生效；否则 CH6 退回增量式 */
static int16_t s_grip_ch0;  /* 第一帧锁存的 CH6 值：|ch−ch0|>EPS 前不动（上电不跳变） */
static bool s_grip_engaged; /* 是否已见到第一次显著变化（= 操作者真动过 Z） */
static int16_t s_grip_command_ch;
static bool s_grip_command_engaged;
static bool s_grip_auto;
static bool s_grip_contact_configured;
static float s_grip_contact_close;
static float s_grip_contact_measured;

void rc_set_grip_range(float open_tick, float close_tick)
{
    s_grip_open = open_tick;
    s_grip_close = close_tick - RC_GRIP_PRESS; /* 越过机械闭合位 → 恒定夹持力 */
    s_grip_cfg = true;
    s_grip_contact_configured=false;
}

bool rc_grip_active(void) { return s_grip_cfg; }

bool rc_grip_closed_requested(void)
{
    /* Step 0 is -1; step 1 is -0.6. Do not treat the startup -1 holding
     * value as a grasp until the operator has actually engaged the gripper.
     * This reports a close command, not proof of physical object contact. */
    return s_grip_command_engaged && s_grip_command_ch >= -RT_CH_SCALE &&
           s_grip_command_ch <= -(RT_CH_SCALE * 9 / 10);
}

bool rc_grip_preserve_hold_error(void)
{
    /* Existing absolute control always preserves its deliberate press error.
     * The incremental fallback must also retain torque on an intentional
     * close, even when an object prevents reaching the commanded position. */
    return s_grip_cfg || rc_grip_closed_requested();
}

bool rc_grip_auto_command(rc_joint_t *j,bool closed) {
    if(!j||!s_grip_cfg)return false;
    s_grip_auto=true;s_grip_engaged=s_grip_command_engaged=true;
    s_grip_command_ch=closed?-RT_CH_SCALE:RT_CH_SCALE;
    j[RC_GRIP_CH].ch_init=true;
    float destination=closed?(s_grip_contact_configured?s_grip_contact_close:s_grip_close):s_grip_open;
    float error=destination-j[RC_GRIP_CH].cmd_pos;
    float speed=closed&&s_grip_contact_configured?300:RC_GRIP_TPS;
    float deadband=closed&&s_grip_contact_configured?3:RC_GRIP_STOP_DZ;
    j[RC_GRIP_CH].rate=fabsf(error)<=deadband?0:(error>0?speed:-speed);
    return true;
}
bool rc_grip_auto_set_contact(float contact,float preload) {
    if(!s_grip_cfg||rc_grip_closed_requested()||!isfinite(contact)||!isfinite(preload)||
        preload<20||preload>80||contact<s_grip_close+RC_GRIP_PRESS||contact>s_grip_open||
        contact-preload<s_grip_close+RC_GRIP_PRESS)return false;
    s_grip_contact_close=contact-preload;s_grip_contact_measured=contact;
    s_grip_contact_configured=true;return true;
}
bool rc_grip_auto_open_to(rc_joint_t *j,float target) {
    if(!j||!s_grip_cfg||!isfinite(target)||target<s_grip_close+RC_GRIP_PRESS||target>s_grip_open)return false;
    s_grip_auto=true;s_grip_engaged=s_grip_command_engaged=true;
    s_grip_command_ch=RT_CH_SCALE;j[RC_GRIP_CH].ch_init=true;
    float error=target-j[RC_GRIP_CH].cmd_pos;
    j[RC_GRIP_CH].rate=fabsf(error)<=3?0:(error>0?300:-300);
    return true;
}
float rc_grip_floor(void){return s_grip_cfg?s_grip_close+RC_GRIP_PRESS:NAN;}
void rc_grip_auto_hold_current(rc_joint_t *j) {
    s_grip_auto=true;s_grip_engaged=s_grip_command_engaged=false;
    s_grip_command_ch=0;
    if(j){j[RC_GRIP_CH].rate=j[RC_GRIP_CH].vel=0;}
}
bool rc_grip_auto_close_to(rc_joint_t *j,float target,float speed) {
    if(!j||!s_grip_cfg||!isfinite(target)||!isfinite(speed)||speed<20||speed>100||
       target<rc_grip_floor()||target>s_grip_open||target>j[RC_GRIP_CH].cmd_pos+3)return false;
    s_grip_auto=true;s_grip_engaged=s_grip_command_engaged=true;
    s_grip_command_ch=-RT_CH_SCALE;j[RC_GRIP_CH].ch_init=true;
    j[RC_GRIP_CH].rate=j[RC_GRIP_CH].cmd_pos-target<=3?0:-speed;
    return true;
}
bool rc_grip_auto_hold_ready(const rc_joint_t *j) {
    return j&&s_grip_cfg&&s_grip_contact_configured&&j[RC_GRIP_CH].meas_ok&&
        fabsf(j[RC_GRIP_CH].cmd_pos-s_grip_contact_close)<=4&&
        fabsf(j[RC_GRIP_CH].meas-s_grip_contact_measured)<=16&&fabsf(j[RC_GRIP_CH].rate)<1;
}
void rc_grip_auto_freeze(rc_joint_t *j) {
    if(j){j[RC_GRIP_CH].rate=0;j[RC_GRIP_CH].vel=0;}
}
void rc_grip_auto_release(void){s_grip_auto=false;s_grip_engaged=false;}

static bool grip_axis(int i) { return i == RC_GRIP_CH && s_grip_cfg; }

void rc_apply_ctrl(const rt_ctrl_t *c, rc_joint_t *j)
{
    /* Observe operator intent independently of EEPROM/absolute-grip setup:
     * a working incremental fallback can still receive a valid step-0 close. */
    if(!s_grip_auto)s_grip_command_ch = c->ch[RC_GRIP_CH];
    if (!j[RC_GRIP_CH].ch_init&&!s_grip_auto) s_grip_command_engaged = false;
    else if (c->buttons & RT_BTN_Z) s_grip_command_engaged = true;
    for (int i = 0; i < RC_NJ; i++) {
        int16_t ch = c->ch[i];

        if(i==RC_GRIP_CH&&s_grip_auto)continue;
        if (grip_axis(i)) {
            /* ---- CH6 档位制绝对位置：通道值 q ∈ [-1,+1] 在 [闭合, 张开] 间线性插值 ---- */
            if (!j[i].ch_init) {
                /* 第一帧：锁存通道值、不动。遥控端上电恒发抱持值，
                 * 那不是"要动爪"的意图 —— 见 rt_channels.c 的 s_grip_hold 说明。 */
                j[i].ch_init = true;
                j[i].last_ch = ch;
                j[i].rate = 0.0f;
                s_grip_ch0 = ch;
                s_grip_engaged = false;
                continue;
            }
            j[i].last_ch = ch;
            if (!s_grip_engaged) {
                /* ★ 2026-10-03 改用**按键位**判据（原来用幅度 |ch-ch0|>EPS 会失效）：
                 * 档位 0（夹紧）的通道值 q=-1 = -32767，正好等于上电抱持值 s_grip_ch0，
                 * 于是"开机后直接长按夹紧"时 |ch-ch0|=0，幅度判据永不触发 → 爪子合不上。
                 * 现在：只要收到 Z 按下(RT_BTN_Z)，就认定操作者真动了夹爪 → engage。
                 * 上电时遥控端 buttons=0（没按 Z），仍然保持上电位置不夹手。 */
                if (!(c->buttons & RT_BTN_Z)) {
                    j[i].rate = 0.0f; /* 还没按过 Z：保持上电位置 */
                    continue;
                }
                s_grip_engaged = true; /* 第一次按下 Z = 操作者真动了 */
            }
            /* q = ch/SCALE ∈ [-1,+1] → 线性插值成绝对目标 tick。
             * q=-1 → close（夹紧，含 PRESS 越过量），q=+1 → open（全张开）。 */
            float q = (float)ch / (float)RT_CH_SCALE;
            q = clampf(q, -1.0f, 1.0f);
            float target = s_grip_close + (s_grip_open - s_grip_close) * (q + 1.0f) * 0.5f;
            float err = target - j[i].cmd_pos;
            if (err > RC_GRIP_STOP_DZ) {
                j[i].rate = RC_GRIP_TPS;
            } else if (err < -RC_GRIP_STOP_DZ) {
                j[i].rate = -RC_GRIP_TPS;
            } else {
                j[i].rate = 0.0f; /* 到位：停。误差留在 cmd 与 meas 之间 = 夹持力 */
            }
            continue;
        }

        if (RC_RATE_CH[i]) {
            /* ---- 速率式：通道绝对值 = 速度指令 ---- */
            if (!j[i].ch_init) {
                /* 第一帧不产生速度 —— 保住"上电不跳变"（哪怕此刻摇杆是推着的，
                 * 也要等链路稳定后的下一帧才开始动）。 */
                j[i].last_ch = ch;
                j[i].ch_init = true;
                j[i].rate = 0.0f;
                continue;
            }
            j[i].last_ch = ch;

            float q = (float)ch / (float)RT_CH_SCALE;
            float aq = fabsf(q);
            if (aq < RC_RATE_DZ) {
                q = 0.0f; /* 死区内 = 停（杀 yaw 漂移，实测 q≈0.028） */
            } else {
                /* 死区外重新拉满量程：让 rate 在死区边缘连续（从 0 起步，不跳变） */
                q = (q > 0.0f ? q - RC_RATE_DZ : q + RC_RATE_DZ) / (1.0f - RC_RATE_DZ);
            }
            j[i].rate = clampf(q * RC_RATE_FULL_TPS, -RC_RATE_FULL_TPS, RC_RATE_FULL_TPS);
            continue;
        }

        /* ---- 增量式（CH3/CH4，及夹爪限位读不出时的回退）：帧间差累加 ---- */
        if (!j[i].ch_init) {
            /* 第一帧只记基准，不产生增量 —— 这保证"上电不跳变"：
             * 无论遥控端此刻的通道值是多少，臂都从当前位置开始，位移为 0。 */
            j[i].last_ch = ch;
            j[i].ch_init = true;
            j[i].carry = 0.0f;
            continue;
        }
        /* 帧间差 = 这一帧操作者想让轴走多少 */
        int32_t dch = (int32_t)ch - (int32_t)j[i].last_ch;
        j[i].last_ch = ch;

        /* 折算成 tick + **上帧被钳掉的余量**（carry）。
         * ★ 2026-10-03 实测修复：没有 carry 时，手腕快速转动每帧只想兑现 12 tick、
         * 其余位移**永久丢失**（通道到顶后差值归零，账就死了）——
         * 表现就是"CH4 角度不够、不灵活，转得越快丢得越多"。
         * 有 carry 后单帧上限只是斜率限制，位移分文不少、分期兑现。 */
        float want = (float)dch / (float)RT_CH_SCALE * (RC_CH_SPAN_TICKS / 2.0f) + j[i].carry;
        float dw = clampf(want, -RC_DW_DELTA_MAX, RC_DW_DELTA_MAX);
        j[i].carry = want - dw;

        /* ★ 只累加在**指令位置**上，不读实测、不设限幅 —— 见顶部说明② */
        j[i].cmd_pos += dw;
    }
}

void rc_step(rc_joint_t *j, float dt_ms)
{
    /* ★ 速率通道：把 rate 按 dt 积分进 cmd_pos（累加基准仍是 cmd_pos，不是 meas）。
     * 增量通道：cmd_pos 已经在 rc_apply_ctrl 里更新完，这里什么都不做 ——
     * 轨迹的连续性由**输入**保证（每帧增量上限 RC_DW_MAX_TICKS）。 */
    float dt_s = dt_ms / 1000.0f;
    for (int i = 0; i < RC_NJ; i++) {
        if (!RC_RATE_CH[i] && !grip_axis(i)) {
            j[i].vel = 0.0f;
            continue;
        }
        float dw = j[i].rate * dt_s;
        /* 单帧上限兜底：正常推杆到不了（800 tps × 5ms = 4 tick < 12），
         * 只防"某帧 rate 被垃圾数据打飞"之类的猛冲。 */
        dw = clampf(dw, -RC_DW_MAX_TICKS, RC_DW_MAX_TICKS);
        j[i].cmd_pos += dw;
        j[i].vel = j[i].rate;

        /* ★ 夹爪不做反缠绕/松手收敛：闭合目标**故意**在机械限位外 RC_GRIP_PRESS，
         * cmd 与 meas 的常驻误差就是夹持力本身，收敛逻辑会把它抹掉。
         * （顶限位发热由"目标是绝对位置、不会无限积分"结构性排除。） */
        if (grip_axis(i)) {
            continue;
        }

        /* ★ 反缠绕（anti-windup）：顶住机械限位时实测不再动、cmd_pos 还在积分，
         * 差超过 RC_WINDUP_TICKS(200) 就每帧往回收 RC_WINDUP_PULL(2)，
         * 把误差稳定在 200（> RC_STALL_TICKS=180 → 堵转保护仍能触发松力矩）。
         *
         * ⚠️⚠️ 2026-10-03 实测教训：**绝不能把 cmd_pos 收敛到实测位置**（曾经的
         * "松手收敛"已删）。重力负载的关节（CH2 抬着整条臂）靠"指令与实测的
         * 常驻误差"产生保持力矩 —— 收敛逻辑把误差抹平 = 卸掉保持力矩 = 臂下垂，
         * 下垂又产生新误差又被抹平 → 正反馈，实测 CH2 缓缓下垂 + 抖动（拉锯痕迹）。
         * 静止时的 cmd−meas 误差是**载荷的像**，不是该消除的偏差。
         * 顶限位松手后的持续顶力（发热）交给堵转保护与操作者，不用收敛换。 */
        /* ★ 2026-10-03 操作者实测修复：**反缠绕只在"还推着杆(rate≠0)"时收，
         * 一松手(rate=0)立刻冻结 cmd_pos，绝不再往实测拖。**
         * 原来无论推没推都在跑：松手后它继续每帧把 cmd_pos 往 meas 收 2 tick，
         * 舵机目标被一路拉低 → 臂自己慢慢往回走 = 操作者说的"像有个积分函数
         * 一直在作用、松手后回中"。而且只在快速/大幅移动(err 曾>200)后才明显，
         * 慢/小移动 err 没过 200 就不回 → "时而回中时而不回"。
         * 冻结后：松手 = 停在原地不回退。顶限位 runaway 仍由"推着时反缠绕 +
         * 堵转保护(err>180 持续 1.5s 松力矩)"两道兜底，安全不受影响。 */
        if (j[i].meas_ok && j[i].rate != 0.0f) {
            float err = j[i].cmd_pos - (float)j[i].meas;
            if (err > RC_WINDUP_TICKS || err < -RC_WINDUP_TICKS) {
                float target = (float)j[i].meas + (err > 0.0f ? RC_WINDUP_TICKS
                                                              : -RC_WINDUP_TICKS);
                j[i].cmd_pos += clampf(target - j[i].cmd_pos, -RC_WINDUP_PULL, RC_WINDUP_PULL);
            }
        }
    }

}

void rc_zero_rate(rc_joint_t *j)
{
    /* 失联/急停：速率通道（含夹爪双态）必须立刻归零 ——
     * 推住的摇杆/按住的 Z 键不能变成"失联后继续跑"。
     * 增量通道无此问题（没有新帧就没有增量）。 */
    for (int i = 0; i < RC_NJ; i++) {
        if (RC_RATE_CH[i] || grip_axis(i)) {
            j[i].rate = 0.0f;
        }
    }
}
