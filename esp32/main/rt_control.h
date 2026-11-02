/*
 * rt_control.h -- 接收端控制律：**增量式**位置控制
 * ============================================================================
 * ★★ 2026-10-02 晚定案：改用增量式（incremental），取代原来的绝对映射 ★★
 *
 * 这是操作者提出的方案，而且是对的。原架构：`目标 = 锚点 + 通道值 × 行程`，
 * 而"行程"只能靠猜（标称 ±1000，或 EEPROM 行程/4）。今天一半的 bug 都出在
 * 这个猜出来的数上：腕部撞死、底座单向 —— 都是"指令量程 > 真实行程"。
 *
 * 增量式：`目标 = 指令位置 + 摇杆增量`
 *   ① **撞限位自动停止**：撞上后位置读数不再增加 → 下一周期增量加不上去
 *      → 自己就不顶了。这是**结构性**保证，不靠参数调对。
 *   ② **不需要软限位**：整套 limit band / channel_half / 行程自测都删掉了 ——
 *      它们正是复杂度与 bug 的来源。
 *   ③ **不需要标定**：只依赖"位置会变"这一个事实，与轴的绝对范围无关。
 *
 * ⚠️ 时序纪律（增量式的经典坑，必须守住）：
 *    累加基准是**我们自己的指令位置 cmd_pos**，不是实测位置 meas。
 *    若拿实测当基准，舵机还没走到位时下一帧会基于旧位置再加一次
 *    → 误差累积 → **越加越快飞车**。
 *
 * 两道兜底（增量式仍需要，因为"卡住"和"限位"外观一样）：
 *   · 每轴增量上限：单帧最多走 DW_MAX_TICKS，防止某帧通道跳变导致猛冲；
 *   · 堵转保护：指令在走、实测长时间不动 → 松那一轴的力矩（见主循环）。
 * ============================================================================
 */
#ifndef RT_CONTROL_H
#define RT_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "rt_protocol.h"

#define RC_NJ 6 /* 关节数 */

/* 控制周期：5 ms = 200 Hz */
#define RC_PERIOD_MS 5

typedef struct {
    float cmd_pos; /* ★ 我们的**指令位置**（tick，浮点保留小数）。
                    * 累加基准 —— 绝不能用实测位置，见顶部时序说明。 */
    float vel;     /* 当前速度（tick/s），用于加速度限制 */
    float rate;    /* ★ 速率通道的速度设定（tick/s）。通道值 = 速度指令，
                    * 由 rc_apply_ctrl 更新、rc_step 按 dt 积分。仅 CH1/2/5 使用。 */
    int16_t meas;  /* 最近一次读到的实测位置（tick），只用于遥测与堵转判断 */
    bool meas_ok;  /* 该轴最近一次回读是否成功 */
    int16_t last_ch; /* 上一帧该通道的值，用于求增量 */
    bool ch_init;    /* 是否已初始化 last_ch（第一帧不产生增量） */
    float carry;     /* ★ 增量通道的"余量结转"：被单帧上限钳掉的位移记在账上，
                      * 下一帧继续兑现 —— **一次都不丢**。没有它，快速转手腕时
                      * 每帧只兑现 12 tick、其余永久丢失，实测表现就是
                      * "CH4 角度不够、控制不灵活"（转得越快丢得越多）。 */
} rc_joint_t;

/* ★★ 2026-10-02 深夜（第三轮实测后）：CH1/CH2/CH5 从"增量位置"改为**速率式** ★★
 * 实测反馈：增量位置式下摇杆是"弹簧偏移"—— 推到底 = 一次性累加固定位移就停，
 * 松手 = 原路退回；想持续转只能"推-松-推"地泵。而且快速推动时，位移增量被
 * RC_DW_MAX_TICKS(12/帧) 钳掉大半（推满一趟只剩 ~60 tick ≈ 5°），
 * 现场表现就是"只能在很小幅度内控制"。
 * 速率式 = 标准摇杆语义：**推住 = 关节持续匀速转、推得越深转得越快、松手 = 立刻停**。
 * 通道值语义变为速度：q = ch/RT_CH_SCALE，rate = q × RC_RATE_FULL_TPS。
 * ★ 2026-10-03：CH3/CH4 也并入速率式（操作者不要"手腕回正 = 关节回退"的镜像语义），
 * 现在 CH1..CH5 全部速率式，只有 CH6（夹爪）是双态绝对 —— 它要的就是两个确定位置。
 * 防撞限位：见 RC_WINDUP_TICKS 反缠绕说明（rt_control.c 的 rc_step）。 */
#define RC_RATE_FULL_TPS 800.0f /* 满偏转速度（tick/s）：800 ≈ 70°/s，全行程 ~3 秒 */
#define RC_RATE_DZ 0.03f        /* 速率死区：杀掉 yaw 漂移（实测 ≈3.4°/min → q≈0.028）*/
#define RC_WINDUP_TICKS 200.0f  /* 反缠绕带宽：cmd_pos 与实测差超过它就往回收。
                                 * 取 200 > RC_STALL_TICKS(180)：真顶死限位时误差停在
                                 * 200，堵转保护仍能触发松力矩 —— 安全性不变。 */
#define RC_WINDUP_PULL 2.0f     /* 每帧最多回收 2 tick：读数偶发串位/垃圾值时，
                                 * cmd_pos 只被缓慢拉动，不会跟着跳变（防误伤）。 */
/* ⚠️ RC_RELAX_FRAMES（"松手收敛"）已删除 —— 2026-10-03 实测：把 cmd_pos 收敛到
 * 实测会抹掉重力关节的保持误差（误差 = 保持力矩的来源），CH2 缓缓下垂 + 抖动。
 * 静止时的 cmd−meas 差是**载荷的像**，不许自动消除。见 rt_control.c rc_step 注释。 */

/* 把全部速率通道归零（失联/急停时调用：推住的摇杆不能变成"失联后继续跑"） */
void rc_zero_rate(rc_joint_t *j);

/* ★★ 2026-10-03：CH6 夹爪 = **档位制绝对位置**（multi-step absolute）★★
 * 演进：增量式（合不拢/夹不紧）→ 双态绝对（开/合两档）→ **档位制**（6 档）。
 * 原增量式的三个结构性 bug（实测）：
 *   ① Z 键 0.5s 斜坡被 RC_DW_MAX_TICKS 钳到只剩 ~300/1400 tick，斜坡走完差值归零，
 *      **被钳掉的位移永久丢失** —— 每按一下只走 20% 行程；
 *   ② "合" = 回到上电起点，而起点离真闭合位（EEPROM limit_min）差 161 tick
 *      —— 所以"合上的时候最后还是有点宽"；
 *   ③ 夹持力需要**恒定位置误差**（指令越过实际），增量式到位即停、误差=0 → 夹不紧。
 *
 * 档位制：遥控端把 Z 手势映射成 GRIP_STEPS 个档（0=夹紧，1..N-1=逐级张开），
 * 通道值 q = -1 + 2×step/(N-1) ∈ [-1,+1]。接收端把 q 在 [闭合位, 张开位] 之间
 * **线性插值**成绝对目标：target = close + (open − close)×(q+1)/2。
 *   · open  = EEPROM limit_max（全张开）
 *   · close = EEPROM limit_min − RC_GRIP_PRESS（故意越过机械闭合位 = 恒定夹持力）
 * 好处：既能"张开一点点"精细夹不同尺寸的物体，又保留 0 档一键夹紧；
 *       每一档都是**确定绝对位置**，与增量式的"位移丢失"彻底无关。
 * 配套纪律（都在 rt_control.c / rt_rx_main.c 落实）：
 *   · 上电不跳变：锁存第一帧的 CH6 值 s_grip_ch0，**通道值显著变化之前不动**
 *     （遥控端上电恒发抱持值，那不是"要动爪"的意图，见 rt_channels.c）。
 *     ⚠️ 档位制下 1 档仍是负值（q=-0.6），旧的"符号翻转"判据永不触发 ——
 *     改用 |ch − ch0| > RC_GRIP_ENGAGE_EPS 的**幅度**判据。
 *   · joint 6 不做反缠绕/松手收敛（会把夹持误差抹掉）；
 *   · joint 6 从堵转保护里排除（夹住物体 = 误差常驻且不动，是**意图**不是故障，
 *     松力矩 = 当场丢件）。 */
#define RC_GRIP_TPS 1200.0f   /* 夹爪开合速度（tick/s）：全行程 ~1.2 s */
#define RC_GRIP_PRESS 50.0f   /* 闭合越过量（tick ≈ 4.4°）：夹持力的来源。
                               * 空夹时顶住机械限位 50 tick，torque_max=500 下
                               * 只是微温；夹物体时误差 = 物宽 + 50 → 夹得更紧。 */
#define RC_GRIP_STOP_DZ 15.0f /* 到位停带：|cmd−target| 小于它就 rate=0，防来回蹭 */
#define RC_GRIP_ENGAGE_EPS 3000 /* 首次动作判据：|ch − 上电锁存值| 超过它才算
                                 * "操作者真动了 Z"。1 档 ≈ -0.6×32767 ≈ -19660，
                                 * 与抱持值 -32767 差 ~13000 ≫ 3000 → 稳定触发；
                                 * 通道噪声/抖动 < 3000 → 不误触发。 */
#define RC_GRIP_CH 5          /* 夹爪的关节下标（CH6 = joint 6 = index 5） */

/* 总线扫描读到 joint 6 的 EEPROM 限位后调用一次：
 * open_tick = limit_max，close_tick = limit_min（PRESS 在内部扣）。
 * 没调用（限位读不出）时 CH6 退回原增量式 —— 能用但行程打折，启动日志会警告。 */
void rc_set_grip_range(float open_tick, float close_tick);

/* 夹爪双态绝对模式是否已生效（堵转保护用它决定是否跳过 joint 6） */
bool rc_grip_active(void);
bool rc_grip_auto_command(rc_joint_t *j,bool closed);
bool rc_grip_auto_open_to(rc_joint_t *j,float target);
bool rc_grip_auto_close_to(rc_joint_t *j,float target,float speed);
float rc_grip_floor(void);
void rc_grip_auto_hold_current(rc_joint_t *j);
/* One observed object's measured contact position, with 20..80 tick preload.
 * RAM only, automatic closure only; must configure with open jaws. */
bool rc_grip_auto_set_contact(float contact,float preload);
bool rc_grip_auto_hold_ready(const rc_joint_t *j);
void rc_grip_auto_freeze(rc_joint_t *j);
void rc_grip_auto_release(void);
/* Engaged step-0 close request; false for the untouched startup hold value.
 * The command is latched by the remote and does not require Z to stay down. */
bool rc_grip_closed_requested(void);
/* Preserve intentional gripper position error as holding effort, including
 * engaged step-0 operation when EEPROM limits forced incremental fallback. */
bool rc_grip_preserve_hold_error(void);

/* 每个通道的"满行程折算"：ch 从 -RT_CH_SCALE 走到 +RT_CH_SCALE
 * 对应多少 tick 的**总位移**。这是手感参数（快慢），不是限位 ——
 * 设大了臂走得快，设小了走得慢，都不会"撞死"。 */
#define RC_CH_SPAN_TICKS 1400.0f

/* 单帧最大增量（tick）：兜住"某帧通道跳变"造成的猛冲。
 * 200 Hz 下 12 tick/帧 = 2400 tick/s ≈ 211 °/s，比人手快推还快，正常不生效。 */
#define RC_DW_MAX_TICKS 12.0f

/* 增量通道（CH3/CH4，50 Hz 兑现）的单帧上限：30 tick/帧 = 1500 tick/s ≈ 132°/s。
 * 配合 carry 结转，这只是**斜率限制**（防猛冲），不再造成位移丢失。
 * 全行程 1400 tick ≈ 0.9 s 兑现完 —— 手腕快甩也能全额到位。 */
#define RC_DW_DELTA_MAX 30.0f

/* 加速度限制（tick/s²）：让起停是斜坡而不是阶跃（"无卡顿"） */
#define RC_AMAX 40000.0f

/* 到位死区（tick）：误差小于它就不动，避免静止微抖 */
#define RC_DEADBAND 3.0f

/* 用一帧控制值更新目标：**求增量并累加到 cmd_pos**（见顶部说明） */
void rc_apply_ctrl(const rt_ctrl_t *c, rc_joint_t *j);

/* 一个控制周期：把 cmd_pos 朝目标平滑推进（加速度限制 + 到位死区）。
 * 增量式下 cmd_pos 本身就是目标，这里只做速度整形。 */
void rc_step(rc_joint_t *j, float dt_ms);

/* 用实测位置对齐指令位置（上电时调一次：让"指令 = 实测"，臂纹丝不动） */
void rc_sync_to_meas(rc_joint_t *j);
typedef struct { int64_t since_us; int16_t reference; } rc_stall_guard_t;
bool rc_stall_step(rc_stall_guard_t *s,bool monitor,int64_t now_us,float command,
    int16_t measured,float error_limit,unsigned timeout_ms,int movement_limit);

#endif /* RT_CONTROL_H */
