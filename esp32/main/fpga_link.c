/*
 * fpga_link.c -- FPGA 视觉桥实现
 * ============================================================================
 * 设计上的三条硬约束（都是现场踩坑换来的）：
 *
 * ① **默认不接管**。FPGA 开环时 P/E 长期饱和 ±4，接上臂就是约 53°/s 持续跑。
 *    所以本模块上电只收数据、只打印，`fpga_link_get()` 在未使能时恒 false；
 *    必须有人在控制台敲 `fpga on`。这条比任何解析优化都重要。
 *
 * ② **坏帧绝不续命**。只有严格通过 35 字节格式、坐标范围、行尾 CRLF、以及
 *    |P|/|E| <= FPGA_INC_MAX 的帧，才更新“最近有效帧”时间戳。
 *    解析失败的帧不能把 stale 计时器重置 —— 否则线断了但噪声还在时，
 *    模块会一直以为链路有效，输出最后一条速度指令。
 *
 * ③ **不另造控制路径**。本模块只产出 rt_ctrl_t，主循环仍走原来的
 *    rc_apply_ctrl()/rc_step()/急停/堵转/同步写。这样安全逻辑只有一份。
 *
 * 控制台用的是 **UART0 + 独立行读取任务**（不是 esp_console）：
 * 本工程一直是裸 printf，没有 console 组件依赖；这里只读 UART0 的 RX，
 * 不碰 stdout 的配置，靠 50 ms 超时轮询，不阻塞控制循环。
 * ============================================================================
 */
#include "fpga_link.h"
#include "fpga_pos.h"
#include "vision_wire.h"
#include "driver/gpio_filter.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#if FPGA_CONSOLE_ENABLE && FPGA_CONSOLE_VIA_USB_JTAG
/* 本板控制台输入在原生 USB-Serial-JTAG 上，UART0 的 GPIO43/44 没引出来。 */
#include "driver/usb_serial_jtag.h"
#endif

/* 控制台必须和 sdkconfig 的 console UART 一致。若以后把主控制台改到
 * USB-Serial-JTAG，这个模块的 `fpga on` 就收不到；届时改 FPGA_CONSOLE_UART
 * 或改用 esp_console，别硬跑。 */
#if defined(CONFIG_ESP_CONSOLE_UART) && CONFIG_ESP_CONSOLE_UART
#if defined(CONFIG_ESP_CONSOLE_UART_NUM) && (CONFIG_ESP_CONSOLE_UART_NUM != 0)
#error "fpga_link: console UART is not UART0; this module's console reader expects UART0"
#endif
#endif

/* 编不过的接线错误，最好在编译期就炸，而不是上电后才发现 UART 抢了。 */
_Static_assert(FPGA_UART != UART_NUM_0, "FPGA UART must not be UART0 (console)");
_Static_assert(FPGA_UART != UART_NUM_1, "FPGA UART must not be UART1 (servo bus)");
_Static_assert(FPGA_FRAME_LEN == 35, "protocol frame is fixed at 35 bytes");
_Static_assert(FPGA_IMG_WIDTH > 0 && FPGA_IMG_WIDTH <= 1000, "coords are 3 digits");
_Static_assert(FPGA_IMG_HEIGHT > 0 && FPGA_IMG_HEIGHT <= 1000, "coords are 3 digits");
_Static_assert(FPGA_TASK_CORE >= 0 && FPGA_TASK_CORE <= 1, "ESP32-S3 has core 0/1");
#if FPGA_CONSOLE_ENABLE
_Static_assert(FPGA_CONSOLE_UART == UART_NUM_0, "console reader expects UART0");
_Static_assert(FPGA_CONSOLE_UART != FPGA_UART, "console and FPGA UART must differ");
#endif
_Static_assert(FPGA_PIN_RX != 17 && FPGA_PIN_RX != 18, "GPIO17/18 belong to the servo bus");
_Static_assert(FPGA_PIN_RX != 43 && FPGA_PIN_RX != 44, "GPIO43/44 belong to the UART0 console");
#if FPGA_PIN_TX >= 0
_Static_assert(FPGA_PIN_TX != FPGA_PIN_RX, "UART2 TX and RX must use different pins");
_Static_assert(FPGA_PIN_TX != 17 && FPGA_PIN_TX != 18, "GPIO17/18 belong to the servo bus");
_Static_assert(FPGA_PIN_TX != 43 && FPGA_PIN_TX != 44, "GPIO43/44 belong to the UART0 console");
#endif

/* ==========================================================================
 * 解析状态机
 * ========================================================================== */
typedef enum {
    FPGA_PS_WAIT_HEADER = 0, /* 等 'R'；其他字节全部丢弃（重新同步） */
    FPGA_PS_IN_FRAME,        /* 已见到 'R'，正在攒 35 字节 */
} fpga_parser_state_t;

static fpga_parser_state_t s_ps = FPGA_PS_WAIT_HEADER;
static uint8_t s_fbuf[VISION_WIRE_LEN];
static uint32_t s_flen = 0;

/* ==========================================================================
 * 共享状态
 * --------------------------------------------------------------------------
 * rx 任务写、200 Hz 控制循环读；控制快照与软限位累计用临界区保护。
 * 统计计数器是“只增不减”的诊断量，用 volatile 单字读写即可。
 * ========================================================================== */
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static gpio_glitch_filter_handle_t s_rx_filter;

static rt_level_request_t s_level_request;
static bool s_level_pending;

static rt_height_request_t s_height_request;
static bool s_height_pending;

bool fpga_link_take_height_request(rt_height_request_t *out)
{
    taskENTER_CRITICAL(&s_lock);
    bool ready = s_height_pending;
    if (ready) {
        *out = s_height_request;
        s_height_pending = false;
    }
    taskEXIT_CRITICAL(&s_lock);
    return ready;
}

static bool height_post(rt_height_request_t request)
{
    taskENTER_CRITICAL(&s_lock);
    bool accepted = !s_height_pending || request.action == HEIGHT_REQ_OFF;
    if (accepted) {
        s_height_request = request;
        s_height_pending = true;
    }
    taskEXIT_CRITICAL(&s_lock);
    return accepted;
}

bool fpga_link_take_level_request(rt_level_request_t *out)
{
    bool ready;
    taskENTER_CRITICAL(&s_lock);
    ready = s_level_pending;
    if (ready) {
        *out = s_level_request;
        s_level_pending = false;
    }
    taskEXIT_CRITICAL(&s_lock);
    return ready;
}

static bool level_post(rt_level_action_t action, float a, float b)
{
    bool accepted;
    taskENTER_CRITICAL(&s_lock);
    /* OFF always wins, including over a pending ON. */
    accepted = !s_level_pending || action == LEVEL_REQ_OFF;
    if (accepted) {
        s_level_request = (rt_level_request_t){action, a, b};
        s_level_pending = true;
    }
    taskEXIT_CRITICAL(&s_lock);
    return accepted;
}

static vision_request_t s_vision_request;
static bool s_vision_pending;
bool fpga_link_take_vision_request(vision_request_t *out) {
    taskENTER_CRITICAL(&s_lock);
    bool ready=s_vision_pending;
    if(ready){*out=s_vision_request;s_vision_pending=false;}
    taskEXIT_CRITICAL(&s_lock);return ready;
}
static bool vision_post(vision_request_t request) {
    taskENTER_CRITICAL(&s_lock);
    bool ok=!s_vision_pending||request.action==4;
    if(ok){s_vision_request=request;s_vision_pending=true;}
    taskEXIT_CRITICAL(&s_lock);return ok;
}

static char s_auto_report[768];
static bool s_auto_report_pending;
void fpga_link_auto_report(const char *text) {
    taskENTER_CRITICAL(&s_lock);
    size_t length=strlen(text);
    if(length<sizeof(s_auto_report)) {
        memcpy(s_auto_report,text,length+1);s_auto_report_pending=true;
    }
    taskEXIT_CRITICAL(&s_lock);
}
static bool s_started = false;
static bool s_enabled = false;
static bool s_ctrl_ready = false;
static int64_t s_ctrl_us = 0;
static rt_ctrl_t s_ctrl;
static uint32_t s_ctrl_seq = 0;

static fpga_frame_t s_last;
static int64_t s_last_frame_us = 0;
static bool s_have_frame = false;

static float s_pan_travel = 0.0f;   /* 单位：ESP32 舵机 tick，不是 FPGA tick */
static float s_reach_travel = 0.0f; /* 同上 */
/* ★ 2026-10-28：软限位拆成两轴各自锁存。
 * 原来共用一个 latch，导致"reach 累满 -> 把 pan 一起清零"，
 * 现象是"跟踪忽然完全不动"。现在谁越界只停谁。 */
static bool s_pan_latched = false;
static bool s_reach_latched = false;
static bool s_soft_latched = false; /* 兼容字段：两轴的 OR，仅用于统计打印 */

/* ---- 2026-10-28: 单通道标定 jog ----------------------------------------
 * 动机：reach 通道"方向对但推不动半径"，靠猜和反复烧固件效率太低。
 * 用 jog 把某个关节单独推到已知量、坚持已知时长，同时记录画面半径/角度，
 * 就能一次量出 d(半径)/d(关节)、d(角度)/d(关节)。
 * pga test 只开 jog、不理 FPGA 帧 —— 标定时视觉伺服不会插进来。 */
static bool    s_jog_ok     = false;
static bool    s_jog_active = false;
static int     s_jog_ch     = 0;
static int16_t s_jog_val    = 0;
static int64_t s_jog_end_us = 0;

/* ★ 2026-10-28：运行时可调增益（百分比，100 = 用编译期 FPGA_G_*）。
 * 动机：reach 的雅可比符号随姿态翻转，台上一改就要重烧固件，效率太低。
 * 现在用 `fpga gain <pan> <sh> <el>` 现场调，0 = 关掉该轴。 */
static volatile int s_gain_pct[3] = {100, 100, 100};

static volatile uint32_t s_rx_bytes = 0;
static volatile uint32_t s_frames_ok = 0;
static volatile uint32_t s_frames_bad = 0;
static volatile uint32_t s_inc_rejected = 0;
static volatile uint32_t s_resyncs = 0;
static volatile uint32_t s_overflows = 0;
static volatile uint32_t s_uart_errors = 0;
static volatile uint32_t s_soft_limit_hits = 0;

static int64_t s_last_echo_us = 0;
static int64_t s_last_inc_warn_us = 0;

/* 静态任务：不引入 malloc/动态栈。ESP-IDF 的栈深度单位是字节。 */
static StaticTask_t s_rx_tcb;
static StackType_t s_rx_stack[FPGA_RX_TASK_STACK];
#if FPGA_CONSOLE_ENABLE
static StaticTask_t s_console_tcb;
static StackType_t s_console_stack[FPGA_CONSOLE_TASK_STACK];
#endif

/* ==========================================================================
 * 小工具
 * ========================================================================== */
static int abs_i(int v) { return v < 0 ? -v : v; }

static bool digits3(const uint8_t *p)
{
    return p[0] >= (uint8_t)'0' && p[0] <= (uint8_t)'9' && p[1] >= (uint8_t)'0' &&
           p[1] <= (uint8_t)'9' && p[2] >= (uint8_t)'0' && p[2] <= (uint8_t)'9';
}

static int dec3(const uint8_t *p)
{
    return ((int)p[0] - (int)'0') * 100 + ((int)p[1] - (int)'0') * 10 +
           ((int)p[2] - (int)'0');
}

/* P/E -> int16 通道值。
 * q = inc × FPS × (舵机 tick / FPGA tick) / 满速 × gain
 * 之后 q×RT_CH_SCALE 再四舍五入。这里刻意不用 rt_q_to_i16()：
 * 它是截断，不是四舍五入；而且这里必须先把 q 换成通道刻度再限幅。 */
static int16_t inc_to_channel(int inc, float gain)
{
    float q = (float)inc * FPGA_TICK_FPS * FPGA_TICK_TO_SERVO * gain / FPGA_TPS_FULL;
    float v = q * (float)RT_CH_SCALE;

    if (v > (float)RT_CH_SCALE) {
        v = (float)RT_CH_SCALE;
    } else if (v < -(float)RT_CH_SCALE) {
        v = -(float)RT_CH_SCALE;
    }
    int32_t r = (v >= 0.0f) ? (int32_t)(v + 0.5f) : (int32_t)(v - 0.5f);
    if (r > RT_CH_SCALE) {
        r = RT_CH_SCALE;
    } else if (r < -RT_CH_SCALE) {
        r = -RT_CH_SCALE;
    }
    return (int16_t)r;
}

/* ==========================================================================
 * 严格帧解析
 * --------------------------------------------------------------------------
 * 为什么不 sscanf("%d,%d ...")：
 *   · 半帧/丢字节时 sscanf 会把后面的字段“错位吃进去”，返回一个看似合法的数；
 *   · 浮点/格式字符串在 -Wformat 下也容易出岔子。
 * 这里逐字节按固定偏移校验：只要有一个字节不对，整帧丢掉，等下一个 'R'。
 * 这样“丢一个字节”最多丢一帧，不会把 x=17 当成 x=217。
 * ========================================================================== */
static bool frame_parse(const uint8_t *b, int len, fpga_frame_t *f)
{
    if (b == NULL || f == NULL || len != FPGA_FRAME_LEN) {
        return false;
    }

    /* 固定骨架：Rxxx,yyy Bxxx,yyy Fxx P±xxx E±xxx\r\n */
    if (b[0] != (uint8_t)'R' || b[4] != (uint8_t)',' || b[8] != (uint8_t)' ' ||
        b[9] != (uint8_t)'B' || b[13] != (uint8_t)',' || b[17] != (uint8_t)' ' ||
        b[18] != (uint8_t)'F' || b[21] != (uint8_t)' ' || b[22] != (uint8_t)'P' ||
        (b[23] != (uint8_t)'+' && b[23] != (uint8_t)'-') || b[27] != (uint8_t)' ' ||
        b[28] != (uint8_t)'E' || (b[29] != (uint8_t)'+' && b[29] != (uint8_t)'-') ||
        b[33] != (uint8_t)'\r' || b[34] != (uint8_t)'\n') {
        return false;
    }

    if (!digits3(&b[1]) || !digits3(&b[5]) || !digits3(&b[10]) || !digits3(&b[14]) ||
        !digits3(&b[24]) || !digits3(&b[30])) {
        return false;
    }

    if ((b[19] != (uint8_t)'0' && b[19] != (uint8_t)'1') ||
        (b[20] != (uint8_t)'0' && b[20] != (uint8_t)'1')) {
        return false;
    }

    int rx = dec3(&b[1]);
    int ry = dec3(&b[5]);
    int bx = dec3(&b[10]);
    int by = dec3(&b[14]);
    if (rx >= FPGA_IMG_WIDTH || ry >= FPGA_IMG_HEIGHT || bx >= FPGA_IMG_WIDTH ||
        by >= FPGA_IMG_HEIGHT) {
        return false; /* 摄像头坐标范围外 = 帧错位或 FPGA 没初始化 */
    }

    int p = dec3(&b[24]);
    int e = dec3(&b[30]);
    if (b[23] == (uint8_t)'-') {
        p = -p;
    }
    if (b[29] == (uint8_t)'-') {
        e = -e;
    }

    f->red_x = (uint16_t)rx;
    f->red_y = (uint16_t)ry;
    f->blue_x = (uint16_t)bx;
    f->blue_y = (uint16_t)by;
    f->found_red = (b[19] == (uint8_t)'1');
    f->found_blue = (b[20] == (uint8_t)'1');
    f->pan_inc = (int16_t)p;
    f->reach_inc = (int16_t)e;
    f->visual_enabled = f->found_red && f->found_blue;
    f->goal_valid = f->visual_enabled;
    f->goal_x=f->blue_x; f->goal_y=f->blue_y;
    f->pole_x=409; f->pole_y=-165;
    f->rx_us = 0; /* 由接收路径填本机时间戳 */
    f->seq = 0;
    return true;
}

/* ==========================================================================
 * 接收 / 控制帧生成
 * ========================================================================== */
static void fpga_echo_frame(const fpga_frame_t *f, const char *raw, int64_t now_us)
{
    if (!fpga_console_connected()) {
        return; /* 主机没开串口：打印只会把接收任务/主循环堵在 USB TX 上 */
    }
    if ((now_us - s_last_echo_us) < (int64_t)FPGA_ECHO_PERIOD_MS * 1000) {
        return;
    }
    s_last_echo_us = now_us;
    long long age_ms = (long long)((now_us - f->rx_us) / 1000);

#if FPGA_ECHO_RAW
    /* raw 是 33 字节可视载荷（已去掉 \r\n），字段一个不改，方便直接抄 R x,y。 */
    printf("[FPGA] %s age=%lldms\n", raw, age_ms);
#else
    (void)raw;
    printf("[FPGA] R%03u,%03u B%03u,%03u F%u%u P%+04d E%+04d age=%lldms\n",
           (unsigned)f->red_x, (unsigned)f->red_y, (unsigned)f->blue_x, (unsigned)f->blue_y,
           f->found_red ? 1u : 0u, f->found_blue ? 1u : 0u, (int)f->pan_inc,
           (int)f->reach_inc, age_ms);
#endif
}

static void fpga_accept_frame(fpga_frame_t *f, const char *raw)
{
    /* ★ P/E 异常：丢弃整帧，而且**不更新时间戳**。否则一个被噪声打飞的 999
     *   既能骗过 stale，又能让下游按满速跑 100 ms。 */
    if (abs_i((int)f->pan_inc) > FPGA_INC_MAX || abs_i((int)f->reach_inc) > FPGA_INC_MAX) {
        s_inc_rejected++;
        int64_t n = esp_timer_get_time();
        if ((n - s_last_inc_warn_us) >= (int64_t)FPGA_REJECT_WARN_MS * 1000) {
            s_last_inc_warn_us = n;
            printf("[FPGA] reject frame: |P|=%d |E|=%d > %d (frame dropped, age NOT refreshed)\n",
                   abs_i((int)f->pan_inc), abs_i((int)f->reach_inc), FPGA_INC_MAX);
        }
        return;
    }

    int64_t now = esp_timer_get_time();
    bool trip = false;
    bool pan_over = false;
    bool reach_over = false;

    taskENTER_CRITICAL(&s_lock);

    s_frames_ok++;
    f->seq = s_frames_ok;
    f->rx_us = now;
    s_last = *f;

    /* UART task publishes data only. Main consumes each seq once after
     * feedback/constraints; it is the sole owner of visual target state. */
    s_last_frame_us = now;
    s_have_frame = true;

    if (s_enabled) {
        /* 运行时增益（百分比）。软限位**必须按实际输出累加**，否则把 reach 增益
         * 设成 0 之后计数器还会涨满并锁存，把 pan 一起拖死。 */
        float gp_acc = (float)s_gain_pct[0] / 100.0f;
        if (!s_pan_latched) {
            /* 累计的是**舵机 tick**：P 的单位是 FPGA tick（≈0.5°），
             * 换算到舵机 tick 才能和 400 tick ≈ 35° 对上。 */
            s_pan_travel += (float)f->pan_inc * FPGA_TICK_TO_SERVO * gp_acc;
            if (fabsf(s_pan_travel) > FPGA_SOFT_LIMIT_TICKS) {
                s_pan_latched = true;
                pan_over = true;
                trip = true;
            }
        }
        {
            float gr_acc = (float)((s_gain_pct[1] > s_gain_pct[2]) ? s_gain_pct[1]
                                                                  : s_gain_pct[2]) / 100.0f;
            if (!s_reach_latched) {
                s_reach_travel += (float)f->reach_inc * FPGA_TICK_TO_SERVO * gr_acc;
                if (fabsf(s_reach_travel) > FPGA_SOFT_LIMIT_TICKS) {
                    s_reach_latched = true;
                    reach_over = true;
                    trip = true;
                }
            }
        }
        if (pan_over || reach_over) {
            s_soft_limit_hits++;
        }
        s_soft_latched = (s_pan_latched || s_reach_latched);

        /* 先整体清零：CH4..CH6、buttons、cmd_code/cmd_seq 全部为 0。
         * CH6=0 是硬要求 —— 非 0 会被 rc_apply_ctrl 当成档位绝对位置。
         * 注意：这里没有新增“FPGA 专用控制路径”，只是填一个普通控制帧。 */
        memset(&s_ctrl, 0, sizeof(s_ctrl));
        s_ctrl.seq = ++s_ctrl_seq;
        s_ctrl.mode = RT_MODE_ARM; /* 最接近“手动/遥控”的既有模式 */
        s_ctrl.cmd_code = RT_CMD_NONE;
        s_ctrl.cmd_seq = 0;
        s_ctrl.buttons = 0;

        {
            /* ★ 2026-10-28 实测：CH1 必须取反。
             * 有界测试记录：P=-4 驱动底座 -728 tick(≈-64°)，画面里标记的极角却
             * 增加了 +63° —— 图像极角 ≈ -(底座角度) + 常数（相机与底座的旋转
             * 方向在图像上是反的）。不取反时闭环正反馈，1.5 s 把角差从 12° 拖到 75°。
             * 极性属于"安装几何"，不是 bug —— 换相机位置要重新确认这一项。 */
            float g0 = FPGA_G_PAN * (float)s_gain_pct[0] / 100.0f;
            float g1 = FPGA_G_SHOULDER * (float)s_gain_pct[1] / 100.0f;
            float g2 = FPGA_G_ELBOW * (float)s_gain_pct[2] / 100.0f;
            if (!s_pan_latched && f->visual_enabled && f->found_red && f->found_blue && f->goal_valid) {
                s_ctrl.ch[0] = inc_to_channel(-(int)f->pan_inc, g0);
            }
            /* ★ 2026-10-28 jog 实测（每档 25% x 0.7s）：
             *   肩 ch2 -25%：j2 -71 tick, 半径 +10.9 px   (dr/dj2 = -0.15 px/tick)
             *   肘 ch3 -25%：j3 -106 tick, 半径 +33.7 px  (dr/dj3 = -0.32 px/tick)
             * 两个关节都是"往负走，半径才变大" —— 所以肩也要取负号。
             * 之前 CH2=+E 时两轴互相抵消，这才是 reach"推不动"的真正原因。 */
            if (!s_reach_latched && f->visual_enabled && f->found_red && f->found_blue && f->goal_valid) {
                s_ctrl.ch[1] = inc_to_channel(-(int)f->reach_inc, g1);
            }
            /* 伸出去 = 肩抬、肘反向，所以 CH3 取负 E。 */
            /* ★ 2026-10-28 实测：肘改同向（原为 -E，肩肘抵消）。
             * 记录：reach 累计 1206 tick(≈105°)，画面半径只 456->470 (+14 px)，
             * 两轴几乎完全对消 —— 说明在该姿态下肩肘需要同号才是"伸出去"。
             * 极性属几何，换姿态/换臂要复核。 */
            /* 2026-10-28 手动扫描实测（见 _reach_sweep 记录）：
             *   肘 j3 +110 tick -> 画面半径 -34 px；肩 j2 -671 tick -> 半径 -75 px
             * 即"肩加、肘减 = 半径变大"，与原始 -E 映射一致，不要改回 +E。 */
            if (!s_reach_latched && f->visual_enabled && f->found_red && f->found_blue && f->goal_valid) {
                s_ctrl.ch[2] = inc_to_channel(-(int)f->reach_inc, g2);
            }
        }
        s_ctrl_us = now;
        s_ctrl_ready = true;
    }

    taskEXIT_CRITICAL(&s_lock);

    if (trip) {
        printf("\n[FPGA] *** SOFT LIMIT (%s) exceeded: pan=%.1f reach=%.1f tick"
               " (limit=%.0f) -> OUTPUT ZERO, latched ***\n",
               (pan_over && reach_over) ? "pan+reach" : (pan_over ? "pan" : "reach"),
               (double)s_pan_travel, (double)s_reach_travel, (double)FPGA_SOFT_LIMIT_TICKS);
        printf("[FPGA] type 'fpga reset' to clear travel/latch, or 'fpga off' to release control.\n");
    }

    fpga_echo_frame(f, raw, now);
}

/* 字节级状态机：
 *   WAIT_HEADER --见到 'R'--> IN_FRAME --攒满且以 \r\n 结束--> 严格解析
 *   IN_FRAME 中再遇到 'R'：认为上一帧是半帧/噪声，立即以新 'R' 重新开始。
 *   超过 35 B 还没见到 '\n'：计数并丢弃（防止串口噪声把缓冲撑爆）。
 * 这样丢字节、掉电重启、半帧、插拔线都不会把状态机永久卡死。 */
static bool wire_seen;
static uint16_t wire_last;
static int64_t wire_time;
static char wire_bad_hex[2*VISION_WIRE_LEN+1];
static unsigned wire_bad_length;
static bool wire_bad_decoded;
static void fpga_fsm_feed(uint8_t b)
{
    s_rx_bytes++;
    if(b=='R'||b=='@') {
        if(s_ps==FPGA_PS_IN_FRAME)s_resyncs++;
        s_ps=FPGA_PS_IN_FRAME;s_flen=0;s_fbuf[s_flen++]=b;return;
    }
    if(s_ps!=FPGA_PS_IN_FRAME)return;
    if(s_flen>=sizeof(s_fbuf)) {
        s_overflows++;s_frames_bad++;s_ps=FPGA_PS_WAIT_HEADER;s_flen=0;return;
    }
    s_fbuf[s_flen++]=b;
    if(b!='\n')return;
    fpga_frame_t f={0};
    bool ok=false;
    bool decoded=false;
    if(s_fbuf[0]=='@') {
        vision_wire_t w;
        if(vision_wire_decode(s_fbuf,s_flen,&w)) {
            decoded=true;
            int64_t now=esp_timer_get_time();
            uint16_t delta=(uint16_t)(w.sequence-wire_last);
            if(!wire_seen||now-wire_time>250000||(delta!=0&&delta<32768)) {
                wire_seen=true;wire_last=w.sequence;wire_time=now;
                f=(fpga_frame_t){.v2=true,.visual_enabled=(w.flags&4)!=0,
                    .key_events_valid=(w.flags&0x80)!=0,.key_events=(w.flags>>5)&3,
                    .goal_valid=(w.flags&16)!=0,.aligned=(w.flags&8)!=0,
                    .found_red=(w.flags&1)!=0,.found_blue=(w.flags&2)!=0,
                    .wire_sequence=w.sequence,.red_x=w.red_x,.red_y=w.red_y,
                    .blue_x=w.blue_x,.blue_y=w.blue_y,.goal_x=w.goal_x,.goal_y=w.goal_y,
                    .pole_x=w.pole_x,.pole_y=w.pole_y,.gap=w.gap,.pan_inc=w.pan,.reach_inc=w.reach};
                ok=true;
            }
        }
    } else ok=frame_parse(s_fbuf,(int)s_flen,&f);
    if(ok) {
        char raw[VISION_WIRE_LEN];
        memcpy(raw,s_fbuf,s_flen-2);raw[s_flen-2]=0;
        fpga_accept_frame(&f,raw);
    } else {
        s_frames_bad++;
        /* Retain diagnostics for the console; no blocking prints in RX. */
        char raw[2*VISION_WIRE_LEN+1];
        for(unsigned i=0;i<s_flen;i++)snprintf(raw+2*i,3,"%02X",s_fbuf[i]);
        taskENTER_CRITICAL(&s_lock);
        memcpy(wire_bad_hex,raw,2*s_flen+1);
        wire_bad_length=s_flen;wire_bad_decoded=decoded;
        taskEXIT_CRITICAL(&s_lock);
    }
    s_ps=FPGA_PS_WAIT_HEADER;s_flen=0;
}

static void fpga_rx_task(void *arg)
{
    (void)arg;
    uint8_t buf[FPGA_RX_CHUNK];

    for (;;) {
        /* 带超时读：任务不会死等；每次最多一行多一点的字节。 */
        int n = uart_read_bytes(FPGA_UART, buf, (uint32_t)sizeof(buf),
                                pdMS_TO_TICKS(FPGA_RX_READ_TIMEOUT_MS));
        if (n < 0) {
            s_uart_errors++;
            vTaskDelay(pdMS_TO_TICKS(FPGA_UART_ERROR_RETRY_MS));
            continue;
        }
        for (int i = 0; i < n; i++) {
            fpga_fsm_feed(buf[i]);
        }
    }
}

/* ==========================================================================
 * 控制台：UART0 行读取（独立任务；只读 RX，不碰 stdout）
 * ========================================================================== */
#if FPGA_CONSOLE_ENABLE
static void fpga_console_height(const char *args)
{
    char sub[16], extra;
    float a, b, c;
    rt_height_request_t request = {0};
    if (args == NULL) goto usage;
    if (strcmp(args, "on") == 0) request.action = HEIGHT_REQ_ON;
    else if (strcmp(args, "off") == 0) request.action = HEIGHT_REQ_OFF;
    else if (strcmp(args, "stat") == 0) request.action = HEIGHT_REQ_STAT;
    else if (sscanf(args, "%15s %f %f %f %c", sub, &a, &b, &c, &extra) == 4 &&
             strcmp(sub, "model") == 0 && isfinite(a) && isfinite(b) &&
             (c == 1 || c == -1)) {
        request = (rt_height_request_t){HEIGHT_REQ_MODEL, a, b, c, 0};
    } else if (sscanf(args, "%15s %f %f %c", sub, &a, &b, &extra) == 3 &&
               strcmp(sub, "limit") == 0 && isfinite(a) && isfinite(b)) {
        request = (rt_height_request_t){HEIGHT_REQ_LIMIT, a, b, 0, 0};
    } else if (sscanf(args, "%15s %f %c", sub, &a, &extra) == 2 &&
               strcmp(sub, "ref") == 0 && isfinite(a)) {
        request = (rt_height_request_t){HEIGHT_REQ_REF, a, 0, 0, 0};
    } else goto usage;
    if (!height_post(request)) printf("[HEIGHT] command busy; retry after acknowledgement\n");
    return;
usage:
    printf("[HEIGHT] fpga height on|off|stat | model <off2> <off3> <sign +1/-1>\n");
    printf("[HEIGHT] fpga height ref <tool-point mm above table> | limit <mm> <margin mm>\n");
    printf("[HEIGHT] model/ref/limit require OFF; ON requires level and calibrated geometry\n");
}

static void fpga_console_level(const char *args)
{
    char sub[16], extra;
    int a, b;
    float trim, value2;
    if (args == NULL) goto usage;
    if (strcmp(args, "on") == 0) {
        if (!level_post(LEVEL_REQ_ON, 0, 0)) goto busy;
    } else if (strcmp(args, "off") == 0) {
        (void)level_post(LEVEL_REQ_OFF, 0, 0);
    } else if (strcmp(args, "stat") == 0) {
        if (!level_post(LEVEL_REQ_STAT, 0, 0)) goto busy;
    } else if (strcmp(args, "source cmd") == 0 || strcmp(args, "source meas") == 0) {
        if (!level_post(LEVEL_REQ_SOURCE, strcmp(args, "source meas") == 0, 0)) goto busy;
    } else if (sscanf(args, "%15s %f %f %c", sub, &trim, &value2, &extra) == 3 &&
               strcmp(sub, "speed") == 0 && isfinite(trim) && isfinite(value2) &&
               trim >= 50 && trim <= 800 && value2 >= 50 && value2 <= 1200) {
        if (!level_post(LEVEL_REQ_SPEED, trim, value2)) goto busy;
    } else if (sscanf(args, "%15s %f %f %c", sub, &trim, &value2, &extra) == 3 &&
               strcmp(sub, "range") == 0 && isfinite(trim) && isfinite(value2) &&
               trim >= 50 && trim <= 1200 && value2 >= 50 && value2 <= 1600) {
        if (!level_post(LEVEL_REQ_RANGE, trim, value2)) goto busy;
    } else if (sscanf(args, "%15s %f %c", sub, &trim, &extra) == 2 &&
               strcmp(sub, "filter") == 0 && isfinite(trim) && trim >= 10 && trim <= 200) {
        if (!level_post(LEVEL_REQ_FILTER, trim, 0)) goto busy;
    } else if (sscanf(args, "%15s %f %c", sub, &trim, &extra) == 2 &&
               strcmp(sub, "tolerance") == 0 && isfinite(trim) && trim >= 0 && trim <= 10) {
        if (!level_post(LEVEL_REQ_TOLERANCE, trim, 0)) goto busy;
    } else if (sscanf(args, "%15s %f %c", sub, &trim, &extra) == 2 &&
               strcmp(sub, "trim") == 0 && isfinite(trim) && fabsf(trim) <= 10.0f) {
        if (!level_post(LEVEL_REQ_TRIM, trim, 0)) goto busy;
    } else if (sscanf(args, "%15s %d %d %c", sub, &a, &b, &extra) == 3 &&
               strcmp(sub, "sign") == 0 && (a == 1 || a == -1) && (b == 1 || b == -1)) {
        if (!level_post(LEVEL_REQ_SIGN, (float)a, (float)b)) goto busy;
    } else goto usage;
    return;
busy:
    printf("[LEVEL] command busy; retry after acknowledgement\n");
    return;
usage:
    printf("[LEVEL] fpga level on|off|stat | trim <-10..10 deg> | sign <+/-1> <+/-1> | source cmd|meas\n");
    printf("[LEVEL] fpga level speed <arm 50..800> <wrist 50..1200> (tick/s)\n");
    printf("[LEVEL] fpga level filter <10..200 ms> | range <arm 50..1200> <wrist 50..1600> (tick)\n");
    printf("[LEVEL] fpga level tolerance <0..10 deg> (modeled pitch slack at wrist bounds)\n");
}

static void fpga_console_usage(void)
{
    printf("[FPGA] commands: fpga on | fpga off | fpga test | fpga stat | fpga reset\n");
    printf("[LEVEL] fpga level on|off|stat | trim <deg> | sign <sh> <el>\n");
    printf("[HEIGHT] fpga height on|off|stat | model <off2> <off3> <sign> | ref <mm> | limit <mm> <margin>\n");
    printf("[FPGA]           fpga jog <ch 1-6> <pct -100..100> <ms 50..1500>\n");
    printf("[FPGA]           fpga gain <pan> <shoulder> <elbow>  (percent, 0..200)\n");
    printf("[FPGA]           fpga pos on|off  (position mode: increment per frame, no dt)\n");
}

static void fpga_console_stat(void)
{
    fpga_link_stats_t st;
    fpga_link_get_stats(&st);
    char bad_hex[sizeof(wire_bad_hex)];unsigned bad_length;bool bad_decoded;
    taskENTER_CRITICAL(&s_lock);
    memcpy(bad_hex,wire_bad_hex,sizeof(bad_hex));
    bad_length=wire_bad_length;bad_decoded=wire_bad_decoded;
    taskEXIT_CRITICAL(&s_lock);
    if(bad_length)printf("[WIREBAD] n=%u decoded=%d raw=%s\n",bad_length,bad_decoded,bad_hex);

    printf("\n[FPGA] enabled=%d ctrl_ready=%d soft_latched=%d have_frame=%d\n", (int)st.enabled,
           (int)st.ctrl_ready, (int)st.soft_limit_latched, (int)st.have_frame);
    printf("[FPGA] rx_bytes=%lu frames ok=%lu bad=%lu inc_rej=%lu resync=%lu overflow=%lu "
           "uart_err=%lu\n",
           (unsigned long)st.rx_bytes, (unsigned long)st.frames_ok,
           (unsigned long)st.frames_bad, (unsigned long)st.inc_rejected,
           (unsigned long)st.resyncs, (unsigned long)st.overflows,
           (unsigned long)st.uart_errors);
    printf("[FPGA] travel pan=%+.1f reach=%+.1f servo-tick | limit=%.0f | soft_hits=%lu\n",
           (double)st.pan_travel, (double)st.reach_travel, (double)FPGA_SOFT_LIMIT_TICKS,
           (unsigned long)st.soft_limit_hits);

    if (st.have_frame) {
        printf("[FPGA] last R%03u,%03u B%03u,%03u F%u%u P%+04d E%+04d age=%lldms seq=%lu\n",
               (unsigned)st.last.red_x, (unsigned)st.last.red_y, (unsigned)st.last.blue_x,
               (unsigned)st.last.blue_y, st.last.found_red ? 1u : 0u,
               st.last.found_blue ? 1u : 0u, (int)st.last.pan_inc, (int)st.last.reach_inc,
               (long long)(st.last_frame_age_us / 1000), (unsigned long)st.last.seq);
    } else {
        printf("[FPGA] last: (no valid frame yet)\n");
    }
    printf("[FPGA] build: auto=%d echo_raw=%d fps=%.1f tick_to_servo=%.4f stale=%dms\n",
           FPGA_LINK_AUTOSTART, FPGA_ECHO_RAW, (double)FPGA_TICK_FPS,
           (double)FPGA_TICK_TO_SERVO, FPGA_STALE_MS);
}

static void fpga_console_jog(const char *args)
{
    int ch = 0, pct = 0, ms = 0;
    if (args == NULL || *args == '\0' || sscanf(args, "%d %d %d", &ch, &pct, &ms) != 3) {
        printf("[FPGA] usage: fpga jog <ch 1-6> <pct -100..100> <ms 50..1500>\n");
        return;
    }
    if (ch < 1 || ch > 6 || pct < -100 || pct > 100 || ms < 50 || ms > 1500) {
        printf("[FPGA] jog: out of range (ch 1-6, pct +-100, ms 50..1500)\n");
        return;
    }
    if (!s_jog_ok) {
        printf("[FPGA] jog refused: type 'fpga test' first (mirror frame control stays OFF)\n");
        return;
    }

    int16_t raw = (int16_t)((int32_t)pct * RT_CH_SCALE / 100);
    int64_t end = esp_timer_get_time() + (int64_t)ms * 1000;

    taskENTER_CRITICAL(&s_lock);
    s_jog_ch = ch - 1;
    s_jog_val = raw;
    s_jog_end_us = end;
    s_jog_active = true;
    taskEXIT_CRITICAL(&s_lock);

    printf("[FPGA] JOG ch=%d (%d pct = %d) for %d ms\n", ch, pct, (int)raw, ms);
}

static void fpga_console_gain(const char *args)
{
    int p = 100, s = 100, e = 100;
    if (args == NULL || *args == '\0' || sscanf(args, "%d %d %d", &p, &s, &e) != 3) {
        printf("[FPGA] usage: fpga gain <pan 0-200> <shoulder 0-200> <elbow 0-200>\n");
        return;
    }
    if (p < 0 || p > 200 || s < 0 || s > 200 || e < 0 || e > 200) {
        printf("[FPGA] gain: 0..200 only (100 = compiled default, 0 = axis off)\n");
        return;
    }
    s_gain_pct[0] = p;
    s_gain_pct[1] = s;
    s_gain_pct[2] = e;
    fpga_pos_set_gain_pct(p, s, e);
    printf("[FPGA] GAIN pan=%d%% shoulder=%d%% elbow=%d%%\n", p, s, e);
}

static void fpga_console_pos(const char *args)
{
    if (args == NULL || *args == '\0') {
        printf("[FPGA] usage: fpga pos on|off\n");
        return;
    }
    if (strcmp(args, "on") == 0) {
        /* ★ 安全默认：只留 pan。reach 的雅可比符号随姿态翻转（见交接 §8.5），
         * 闭环会"越走越偏"，所以位置式一律先关掉肩肘，要用手动 `fpga gain` 开。 */
        s_gain_pct[0] = 100;
        s_gain_pct[1] = 0;
        s_gain_pct[2] = 0;
        fpga_pos_set_gain_pct(100, 0, 0);
        fpga_pos_set_active(true);
        printf("[FPGA]   (safe default gain: pan=100%% shoulder=0%% elbow=0%%)\n");
        printf("[FPGA] POSITION mode ON: absolute targets, one increment per frame (no dt).\n");
        printf("[FPGA]   waits for measured pose to arm; until then the loop holds still.\n");
    } else if (strcmp(args, "off") == 0) {
        fpga_pos_set_active(false);
        printf("[FPGA] POSITION mode OFF: rate mode again\n");
    } else {
        printf("[FPGA] usage: fpga pos on|off\n");
    }
}

static void fpga_console_vision(const char *args,bool grasp) {
    float a,b,c,d,e;char extra;
    vision_request_t r={0};
    if(!args)return;
    if(grasp) {
        if(strcmp(args,"off")==0)r.action=4;
        else if(sscanf(args,"arm %f %c",&a,&extra)==1&&isfinite(a)&&a>=1&&a<=15)r=(vision_request_t){3,a,0,0,0,0};
        else {printf("[PICK] fpga grasp arm <CH2 lift magnitude 1..15 deg> | off\n");return;}
    } else {
        if(strcmp(args,"clear")==0)r.action=2;
        else if(sscanf(args,"add %f %f %f %f %f %c",&a,&b,&c,&d,&e,&extra)==5&&isfinite(a)&&isfinite(b)&&isfinite(c)&&isfinite(d)&&isfinite(e))
            r=(vision_request_t){1,a,b,c,d,e};
        else {printf("[B] fpga b add <CH1 tick> <CH2 tick> <CH3 tick> <dr/dCH2 px/tick> <dr/dCH3 px/tick> | clear\n");return;}
    }
    if(!vision_post(r))printf("[VISION] request busy; wait for acknowledgement\n");
}

static void fpga_console_auto(const char *args) {
    float degrees,contact,preload;char extra;vision_request_t request={0};
    if(!args)return;
    if(strcmp(args,"prepare")==0)request.action=9;
    else if(strcmp(args,"off")==0)request.action=6;
    else if(strcmp(args,"start")==0)request.action=14;
    else if(strcmp(args,"stat")==0)request.action=7;
    else if(sscanf(args,"contact %f %f %c",&contact,&preload,&extra)==2&&
        isfinite(contact)&&isfinite(preload)&&contact>=0&&contact<=4095&&preload>=20&&preload<=80)
        request=(vision_request_t){11,contact,preload,0,0,0};
    else if(sscanf(args,"arm %f %c",&degrees,&extra)==1&&isfinite(degrees)&&degrees>=1&&degrees<=45)
        request=(vision_request_t){5,degrees,0,0,0,0};
    else if(sscanf(args,"finish %f %c",&degrees,&extra)==1&&isfinite(degrees)&&degrees>=1&&degrees<=10)
        request=(vision_request_t){10,degrees,0,0,0,0};
    else if(sscanf(args,"lift %f %c",&degrees,&extra)==1&&isfinite(degrees)&&degrees>=1&&degrees<=45)
        request=(vision_request_t){12,degrees,0,0,0,0};
    else {printf("[AUTO] fpga auto arm <1..45 deg> | finish <1..10 deg> | lift <1..45 deg> | contact <measured tick> <20..80 preload> | off | stat\n");return;}
    if(!vision_post(request))printf("[AUTO] request busy\n");
}
static void fpga_console_probe(const char *args) {
    int channel;float delta;char extra;
    if(!args||sscanf(args,"%d %f %c",&channel,&delta,&extra)!=2||(channel!=2&&channel!=3)||
       !isfinite(delta)||fabsf(delta)>20||fabsf(delta)<0.1f) {
        printf("[PROBE] fpga probe <2|3> <delta -20..20 tick>\n");return;
    }
    if(!vision_post((vision_request_t){8,(float)channel,delta,0,0,0}))printf("[PROBE] request busy\n");
}

static void fpga_console_handle_line(char *line)
{
    char *p = line;
    while (*p == ' ' || *p == '\t') {
        p++;
    }
    if (strncmp(p, "fpga", 4) != 0) {
        return;
    }
    p += 4;
    if (*p != '\0' && *p != ' ' && *p != '\t') {
        return; /* "fpgaX" 不是本模块命令 */
    }
    while (*p == ' ' || *p == '\t') {
        p++;
    }
    if (*p == '\0') {
        fpga_console_usage();
        return;
    }

    char *cmd = p;
    while (*p != '\0' && *p != ' ' && *p != '\t') {
        p++;
    }
    char *args = NULL;
    if (*p != '\0') {
        *p++ = '\0';
        while (*p == ' ' || *p == '\t') {
            p++;
        }
        args = p; /* 可能指向 '\0' */
    }

    if(strcmp(cmd,"auto")==0){fpga_console_auto(args);
    } else if(strcmp(cmd,"probe")==0){fpga_console_probe(args);
    } else if (strcmp(cmd, "level") == 0) {
        fpga_console_level(args);
    } else if (strcmp(cmd, "height") == 0) {
        fpga_console_height(args);
    } else if (strcmp(cmd, "jog") == 0) {
        fpga_console_jog(args);
    } else if (strcmp(cmd, "gain") == 0) {
        fpga_console_gain(args);
    } else if (strcmp(cmd, "pos") == 0) {
        fpga_console_pos(args);
    } else if (strcmp(cmd,"b")==0) {
        fpga_console_vision(args,false);
    } else if (strcmp(cmd,"grasp")==0) {
        fpga_console_vision(args,true);
    } else if (args != NULL && *args != '\0') {
        fpga_console_usage(); /* 其它命令都不带参数 */
    } else if (strcmp(cmd, "on") == 0) {
        fpga_link_set_enable(true);
    } else if (strcmp(cmd, "off") == 0) {
        fpga_link_set_enable(false);
    } else if (strcmp(cmd, "test") == 0) {
        fpga_link_set_test(true);
    } else if (strcmp(cmd, "stat") == 0) {
        fpga_console_stat();
    } else if (strcmp(cmd, "reset") == 0) {
        fpga_link_reset_limits();
    } else {
        fpga_console_usage();
    }
}

bool fpga_console_connected(void)
{
#if FPGA_CONSOLE_ENABLE && FPGA_CONSOLE_VIA_USB_JTAG
    /* 主机没打开串口 = TX FIFO 只进不出，printf 迟早阻塞。 */
    return usb_serial_jtag_is_connected();
#else
    return true; /* UART 控制台：没有"主机不读"这种阻塞语义 */
#endif
}

static void fpga_console_task(void *arg)
{
    (void)arg;
    char line[FPGA_CONSOLE_LINE_MAX];
    size_t len = 0;
    bool over = false;
    uint8_t ch;

    for (;;) {
        // Formatting happens in the main owner; transmit the long diagnostic
        // from this lower-priority console task, never in the control loop.
        char report[768];bool pending_report;
        taskENTER_CRITICAL(&s_lock);
        pending_report=s_auto_report_pending;
        if(pending_report){memcpy(report,s_auto_report,sizeof(report));s_auto_report_pending=false;}
        taskEXIT_CRITICAL(&s_lock);
        if(pending_report) {
#if FPGA_CONSOLE_VIA_USB_JTAG
            size_t sent=0,total=strlen(report);
            while(sent<total) {
                int wrote=usb_serial_jtag_write_bytes(report+sent,total-sent,pdMS_TO_TICKS(100));
                if(wrote<=0)break;
                sent+=(size_t)wrote;
            }
#else
            printf("%s",report);
#endif
        }
#if FPGA_CONSOLE_VIA_USB_JTAG
        /* USB-Serial-JTAG 的读超时单位是 **tick 数**（uint32_t），不是 TickType_t；
         * 这里两种实现的语义要一致，所以都传 pdMS_TO_TICKS()。 */
        int n = usb_serial_jtag_read_bytes(&ch, 1, (uint32_t)pdMS_TO_TICKS(FPGA_CONSOLE_POLL_MS));
#else
        int n = uart_read_bytes(FPGA_CONSOLE_UART, &ch, 1, pdMS_TO_TICKS(FPGA_CONSOLE_POLL_MS));
#endif
        if (n != 1) {
            continue;
        }
        if (ch == (uint8_t)'\r' || ch == (uint8_t)'\n') {
            if (!over && len > 0) {
                line[len] = '\0';
                fpga_console_handle_line(line);
            }
            len = 0;
            over = false;
        } else if (ch == 0x08 || ch == 0x7F) {
            if (len > 0) {
                len--;
            }
        } else if (ch >= 0x20 && ch <= 0x7E) {
            if (len < sizeof(line) - 1) {
                line[len++] = (char)ch;
            } else if (!over) {
                over = true;
                printf("[FPGA] console line too long; discarded until newline\n");
            }
        }
        /* 其他控制字符忽略 */
    }
}

static esp_err_t fpga_console_start(void)
{
#if FPGA_CONSOLE_VIA_USB_JTAG
    /* 从原生 USB-Serial-JTAG 读输入。驱动只装一次；已装过（返回 INVALID_STATE）
     * 视为正常 —— 那是别人先装了，我们照样能读。 */
    if (!usb_serial_jtag_is_driver_installed()) {
        usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
        cfg.tx_buffer_size = 2048;
        cfg.rx_buffer_size = FPGA_CONSOLE_RX_BUF;
        esp_err_t err = usb_serial_jtag_driver_install(&cfg);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            printf("[FPGA] console usb_serial_jtag install failed: %s\n", esp_err_to_name(err));
            return err;
        }
    }
    /* 丢掉上电/下载器留下的噪声：绝不能让一串巧合字节变成 `fpga on`。 */
    {
        uint8_t junk[32];
        while (usb_serial_jtag_read_bytes(junk, sizeof(junk), 0) > 0) {
            /* drain */
        }
    }
#else
    /* 不调用 uart_param_config(UART0)：保持 IDF console 已经配好的波特率/格式，
     * 只装驱动以拿到 RX 环形缓冲。这样不会把正在输出的日志弄乱。 */
    if (!uart_is_driver_installed(FPGA_CONSOLE_UART)) {
        esp_err_t err = uart_driver_install(FPGA_CONSOLE_UART, FPGA_CONSOLE_RX_BUF, 0, 0, NULL, 0);
        if (err != ESP_OK) {
            printf("[FPGA] console UART%d install failed: %s\n", (int)FPGA_CONSOLE_UART,
                   esp_err_to_name(err));
            return err;
        }
    }
    uart_flush_input(FPGA_CONSOLE_UART);
#endif

    TaskHandle_t h = xTaskCreateStaticPinnedToCore(
        fpga_console_task, "fpga_console", (uint32_t)sizeof(s_console_stack), NULL,
        FPGA_CONSOLE_TASK_PRIO, s_console_stack, &s_console_tcb, FPGA_TASK_CORE);
    if (h == NULL) {
        printf("[FPGA] console task create failed\n");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
#else
static esp_err_t fpga_console_start(void) { return ESP_OK; }
#endif /* FPGA_CONSOLE_ENABLE */

/* ==========================================================================
 * 对外接口
 * ========================================================================== */
bool fpga_link_get(rt_ctrl_t *out)
{
    if (out == NULL) {
        return false;
    }

    int64_t now = esp_timer_get_time();
    bool ok = false;

    bool jog_done = false;

    taskENTER_CRITICAL(&s_lock);
    if (s_jog_ok && !s_enabled) {
        /* Test mode always supplies a fresh neutral frame after jog expiry.
         * Otherwise the last jog speed could persist until link timeout. */
        memset(out, 0, sizeof(*out));
        out->seq = ++s_ctrl_seq;
        out->mode = RT_MODE_ARM;
        if (s_jog_active && now < s_jog_end_us) out->ch[s_jog_ch] = s_jog_val;
        if (s_jog_active && now >= s_jog_end_us) {
            s_jog_active = false;
            jog_done = true;
        }
        ok = true;
    } else if (s_jog_active && s_jog_ok) {
        /* jog 优先：单通道定值输出，FPGA 帧不参与（标定用） */
        if (now < s_jog_end_us) {
            memset(out, 0, sizeof(*out));
            out->seq = ++s_ctrl_seq;
            out->mode = RT_MODE_ARM;
            out->cmd_code = RT_CMD_NONE;
            out->ch[s_jog_ch] = s_jog_val;
            ok = true;
        } else {
            s_jog_active = false;
            jog_done = true;
        }
    } else if (s_enabled && s_ctrl_ready) {
        int64_t age = now - s_ctrl_us;
        /* 极少数情况下 RX 任务恰好在取 now 之后更新了 s_ctrl_us，
         * 会让 age 变成 -1..-n µs。不能因为这几微秒就误判 stale：
         * 那会让调用方在刚收到新帧时回落一个控制周期。夹到 0 即可。 */
        if (age < 0) {
            age = 0;
        }
        if (age < (int64_t)FPGA_STALE_MS * 1000) {
            memcpy(out, &s_ctrl, sizeof(*out));
            ok = true;
        }
    }
    taskEXIT_CRITICAL(&s_lock);

    if (jog_done) {
        printf("[FPGA] JOG done\n");
    }
    return ok;
}

void fpga_link_set_test(bool on)
{
    taskENTER_CRITICAL(&s_lock);
    s_jog_ok = on;
    fpga_pos_set_active(false);
    s_enabled = false;      /* test 模式绝不让 FPGA 帧驱动 */
    s_jog_active = false;
    s_ctrl_ready = false;
    s_ctrl_us = 0;
    memset(&s_ctrl, 0, sizeof(s_ctrl));
    taskEXIT_CRITICAL(&s_lock);
    printf("[FPGA] TEST mode %s (jog only, frame control OFF)\n", on ? "ON" : "OFF");
}

void fpga_link_set_enable(bool enable)
{
    bool latched;

    taskENTER_CRITICAL(&s_lock);
    s_enabled = enable;
    s_jog_ok = enable;      /* off 之后 jog 也必须失效 */
    s_jog_active = false;
    /* ★ 2026-10-28 修 bug：`fpga off` 原先只关速率式，位置式仍处于 selected，
     * 主循环就整条跳过 rc_apply_ctrl -> **遥控器完全失控**（现场踩过）。
     * 关闭必须把所有 FPGA 侧输出通道一起关掉。 */
    if (!enable) {
        fpga_pos_set_active(false);
        /* Source OFF does not invalidate the level/table calibration. */
    }
    /* 使能/关闭都作废旧快照：使能后必须等“新帧”，不能拿使能前的帧直接冲。 */
    s_ctrl_ready = false;
    s_ctrl_us = 0;
    memset(&s_ctrl, 0, sizeof(s_ctrl));
    latched = s_soft_latched;
    taskEXIT_CRITICAL(&s_lock);

    if (enable) {
        printf("\n[FPGA] *** ENABLED *** UART%d RX=GPIO%d stale=%dms limit=%.0f tick\n",
               (int)FPGA_UART, FPGA_PIN_RX, FPGA_STALE_MS, (double)FPGA_SOFT_LIMIT_TICKS);
        printf("[FPGA] waiting for a NEW valid frame; fpga_link_get() is false until then.\n");
        if (latched) {
            printf("[FPGA] WARNING: soft limit is still latched -> output will stay 0.\n");
            printf("[FPGA]          type 'fpga reset' first if the arm is in a safe position.\n");
        }
    } else {
        printf("[FPGA] disabled; control falls back to ESP-NOW/HOLD.\n");
    }
}

bool fpga_link_is_enabled(void)
{
    bool en;
    taskENTER_CRITICAL(&s_lock);
    en = s_enabled;
    taskEXIT_CRITICAL(&s_lock);
    return en;
}

void fpga_link_reset_limits(void)
{
    float old_pan;
    float old_reach;
    bool was_latched;

    taskENTER_CRITICAL(&s_lock);
    old_pan = s_pan_travel;
    old_reach = s_reach_travel;
    was_latched = s_soft_latched;
    s_pan_travel = 0.0f;
    s_reach_travel = 0.0f;
    s_pan_latched = false;
    s_reach_latched = false;
    s_soft_latched = false;
    taskEXIT_CRITICAL(&s_lock);

    printf("[FPGA] reset: travel pan=%+.1f reach=%+.1f -> 0; latch %s -> cleared. "
           "Enable state is unchanged (%s).\n",
           (double)old_pan, (double)old_reach, was_latched ? "WAS SET" : "was clear",
           fpga_link_is_enabled() ? "ON" : "OFF");
}

bool fpga_link_frame_fresh(void)
{
    int64_t now = esp_timer_get_time();
    bool ok = false;
    taskENTER_CRITICAL(&s_lock);
    ok = s_have_frame && ((now - s_last_frame_us) < (int64_t)FPGA_STALE_MS * 1000);
    taskEXIT_CRITICAL(&s_lock);
    return ok;
}

bool fpga_link_get_last(fpga_frame_t *out)
{
    if (out == NULL) {
        return false;
    }
    bool ok = false;
    taskENTER_CRITICAL(&s_lock);
    if (s_have_frame) {
        *out = s_last;
        ok = true;
    }
    taskEXIT_CRITICAL(&s_lock);
    return ok;
}

void fpga_link_get_stats(fpga_link_stats_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));

    int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_lock);
    out->enabled = s_enabled;
    out->ctrl_ready = s_ctrl_ready;
    out->soft_limit_latched = (s_pan_latched || s_reach_latched);
    out->have_frame = s_have_frame;
    out->pan_travel = s_pan_travel;
    out->reach_travel = s_reach_travel;
    if (s_have_frame) {
        out->last = s_last;
        int64_t age = now - s_last_frame_us;
        out->last_frame_age_us = (age >= 0) ? age : 0;
    } else {
        out->last_frame_age_us = -1;
    }
    out->rx_bytes = s_rx_bytes;
    out->frames_ok = s_frames_ok;
    out->frames_bad = s_frames_bad;
    out->inc_rejected = s_inc_rejected;
    out->resyncs = s_resyncs;
    out->overflows = s_overflows;
    out->uart_errors = s_uart_errors;
    out->soft_limit_hits = s_soft_limit_hits;
    taskEXIT_CRITICAL(&s_lock);
}

bool fpga_link_send_cmd(int cx, int cy)
{
    if (FPGA_PIN_TX < 0) {
        return false; /* 默认不启用回发；见头文件接线说明 */
    }
    if (!s_started || cx < 0 || cx >= FPGA_IMG_WIDTH || cy < 0 || cy >= FPGA_IMG_HEIGHT) {
        return false;
    }

    char cmd[32];
    int n = snprintf(cmd, sizeof(cmd), "C%d,%d\n", cx, cy);
    if (n <= 0 || n >= (int)sizeof(cmd)) {
        return false;
    }

    int sent = uart_write_bytes(FPGA_UART, cmd, (size_t)n);
    if (sent != n) {
        s_uart_errors++;
        return false;
    }
    return true;
}

/* ==========================================================================
 * 初始化
 * ========================================================================== */
static esp_err_t fpga_uart_init(void)
{
    if (uart_is_driver_installed(FPGA_UART)) {
        printf("[FPGA] *** UART%d already has a driver -- refusing to steal it ***\n",
               (int)FPGA_UART);
        return ESP_ERR_INVALID_STATE;
    }

    uart_config_t cfg = {0};
    cfg.baud_rate = FPGA_BAUD;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_XTAL;

    esp_err_t err = uart_param_config(FPGA_UART, &cfg);
    if (err != ESP_OK) {
        printf("[FPGA] UART%d param failed: %s\n", (int)FPGA_UART, esp_err_to_name(err));
        return err;
    }

    /* TX=-1 时传 UART_PIN_NO_CHANGE，等于不路由 TX；只有 send_cmd() 预留用。
     * RX 固定到 FPGA_PIN_RX，绝不碰 GPIO17/18（舵机总线）。 */
    err = uart_set_pin(FPGA_UART, FPGA_PIN_TX, FPGA_PIN_RX, UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        printf("[FPGA] UART%d set_pin failed: %s\n", (int)FPGA_UART, esp_err_to_name(err));
        return err;
    }

    /* Receive-only isolation test on this installation showed that short
     * input glitches corrupt otherwise complete lines. Hardware filtering
     * rejects sub-two-clock pulses; packet CRC and watchdog remain strict. */
    const gpio_pin_glitch_filter_config_t filter_config={
        .gpio_num=FPGA_PIN_RX,.clk_src=GLITCH_FILTER_CLK_SRC_DEFAULT};
    err=gpio_new_pin_glitch_filter(&filter_config,&s_rx_filter);
    if(err!=ESP_OK)return err;
    err=gpio_glitch_filter_enable(s_rx_filter);
    if(err!=ESP_OK){gpio_del_glitch_filter(s_rx_filter);s_rx_filter=NULL;return err;}

    err = uart_driver_install(FPGA_UART, FPGA_RX_BUF, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        printf("[FPGA] UART%d driver install failed: %s\n", (int)FPGA_UART,
               esp_err_to_name(err));
        return err;
    }
    /* A V2 line is only 59 bytes, below the driver's default FIFO threshold.
     * Deliver short lines promptly, without weakening parsing or freshness. */
    err=uart_set_rx_full_threshold(FPGA_UART,1);
    if(err!=ESP_OK)return err;
    err=uart_set_rx_timeout(FPGA_UART,2);
    if(err!=ESP_OK)return err;
    uart_flush_input(FPGA_UART);
    return ESP_OK;
}

esp_err_t fpga_link_start(void)
{
    if (s_started) {
        return ESP_OK;
    }
    if (!FPGA_LINK_ENABLE) {
        printf("[FPGA] compiled/configured OFF (FPGA_LINK_ENABLE=0)\n");
        return ESP_ERR_NOT_SUPPORTED;
    }

    esp_err_t err = fpga_uart_init();
    if (err != ESP_OK) {
        printf("[FPGA] RX not started; visual bridge stays disabled.\n");
        return err;
    }

    err = fpga_console_start();
    if (err != ESP_OK) {
        printf("[FPGA] *** console unavailable; only FPGA_LINK_AUTOSTART=1 can enable ***\n");
    }

    TaskHandle_t h = xTaskCreateStaticPinnedToCore(
        fpga_rx_task, "fpga_rx", (uint32_t)sizeof(s_rx_stack), NULL, FPGA_RX_TASK_PRIO, s_rx_stack,
        &s_rx_tcb, FPGA_TASK_CORE);
    if (h == NULL) {
        printf("[FPGA] RX task create failed\n");
        return ESP_ERR_NO_MEM;
    }

    s_started = true;

    printf("\n[FPGA] UART%d RX=GPIO%d baud=%d | ", (int)FPGA_UART, FPGA_PIN_RX, FPGA_BAUD);
#if FPGA_PIN_TX >= 0
    printf("TX=GPIO%d", FPGA_PIN_TX);
#else
    printf("TX=disabled");
#endif
    printf(" | frame=%dB @ %.1f fps | tasks on core%d\n", FPGA_FRAME_LEN,
           (double)FPGA_TICK_FPS, FPGA_TASK_CORE);
    printf("[FPGA] mapping: P->CH1 gain %.2f, E->CH2 gain %.2f, -E->CH3 gain %.2f; "
           "CH4..CH6=0\n",
           (double)FPGA_G_PAN, (double)FPGA_G_SHOULDER, (double)FPGA_G_ELBOW);
    printf("[FPGA] 1 FPGA tick=%.4f servo tick; P=4 -> CH1 q=%.4f (~%.0f tick/s); "
           "soft limit=%.0f tick\n",
           (double)FPGA_TICK_TO_SERVO,
           (double)(4.0f * FPGA_TICK_FPS * FPGA_TICK_TO_SERVO * FPGA_G_PAN / FPGA_TPS_FULL),
           (double)(4.0f * FPGA_TICK_FPS * FPGA_TICK_TO_SERVO * FPGA_G_PAN),
           (double)FPGA_SOFT_LIMIT_TICKS);

#if FPGA_LINK_AUTOSTART
    printf("[FPGA] *** AUTOSTART=1: taking over on first new frame after boot ***\n");
    fpga_link_set_enable(true);
#else
#if FPGA_CONSOLE_ENABLE
    printf("[FPGA] DISABLED at boot (safe). Type 'fpga on' to enable; "
           "'fpga stat' / 'fpga off' / 'fpga reset'.\n");
#else
    printf("[FPGA] DISABLED at boot (safe), and FPGA_CONSOLE_ENABLE=0: "
           "no console command can enable it.\n");
#endif
#endif

    return ESP_OK;
}
