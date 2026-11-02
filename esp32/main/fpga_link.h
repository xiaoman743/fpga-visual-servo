/*
 * fpga_link.h -- FPGA 视觉桥：把 FPGA 的 P/E 增量变成 6 通道速度指令
 * ============================================================================
 * 数据流（本模块只做“翻译”，不新增任何控制路径）：
 *
 *   FPGA --UART2 115200 8N1, 35 B/frame, ~26.7 fps--> 本模块
 *        -> 严格 FSM 解析 / 安全门限 / 单位换算
 *        -> rt_ctrl_t（mode=RT_MODE_ARM, CH1..CH5 速率式, CH6 恒 0）
 *        -> 主循环原有的 rc_apply_ctrl()/rc_step()/急停/堵转/同步写
 *
 * ★★ 最重要的安全决定：默认不使能 ★★
 *   实测 FPGA 在没接机械臂（开环）时 P/E 会长期饱和在 ±4。按本文件的比例，
 *   P=4 换算到 CH1 是 q≈0.759，下游 RC_RATE_FULL_TPS(800) 给出约 607 tick/s
 *   ≈ 53°/s。也就是说：**上电即接管 = 机械臂以 53°/s 一直跑**。
 *   所以 FPGA_LINK_AUTOSTART 必须是 0，只有控制台输入 `fpga on` 才使能。
 *
 * ★ 本文件顶部集中所有可调参数。改任何“比例/限幅/时限”都只改这里，
 *   不要把魔数散到 .c 里 —— 现场标定时最怕“改了 A 忘了 B”。
 * ============================================================================
 */
#ifndef FPGA_LINK_H
#define FPGA_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "rt_protocol.h"

/* ==========================================================================
 * 一、总开关
 * ========================================================================== */

/* 编译开关。0 = 整个模块不启动（patch 里的调用也被 #if 掉），
 * 但 fpga_link.c 仍会被编译，所以保持本文件可独立编译。 */
#define FPGA_LINK_ENABLE 1

/* ★★ 上电是否自动使能。**默认必须是 0**，原因见文件头 ★★
 * 只有你确认 FPGA 已经闭环、机械臂周围清空、并且接受“一上电就可能动”时，
 * 才允许改成 1。改成 1 后仍会等待“使能之后的新帧”，不会拿旧帧直接冲。 */
#define FPGA_LINK_AUTOSTART 0

/* ==========================================================================
 * 二、UART / 引脚 / 任务
 * ========================================================================== */

/* FPGA 接 UART2。⛔ 不要用 UART0（控制台）或 UART1（STS3215 舵机总线，
 * TX=17/RX=18/1 Mbps）；三者抢同一个外设会得到“日志乱码/舵机丢帧”这种
 * 看起来像别的问题的故障。 */
#define FPGA_UART UART_NUM_2
#define FPGA_PIN_RX 5  /* FPGA TX -> ESP32-S3 GPIO5；只收，默认不接 TX */
#define FPGA_PIN_TX (-1) /* -1 = 不启用回发。要发 C 指令时改成实际 GPIO */
#define FPGA_BAUD 115200
#define FPGA_RX_BUF 2048 /* UART 驱动环形缓冲；26.7 fps 只占约 1 KB/s，留 2 s 余量 */

/* 接收任务用“带超时的读”，绝不 portMAX_DELAY 死等：
 * 这样任务永远能响应调度/删任务，也不会把 200 Hz 控制循环拖住。
 * 读本身在独立任务里，20 ms 超时只是让任务周期性醒来。
 * 任务绑 core1、优先级低于舵机回读(5)：不和控制循环(core0)、UART1 抢。 */
#define FPGA_RX_READ_TIMEOUT_MS 20
#define FPGA_RX_CHUNK 64        /* 每次 uart_read_bytes 最多取多少字节（内部工作缓冲） */
#define FPGA_UART_ERROR_RETRY_MS 100 /* UART 读返回负值后的退避，防止错误风暴 */
#define FPGA_TASK_CORE 1        /* 主控制循环在 core 0；本模块两个任务都绑 core 1 */
#define FPGA_RX_TASK_PRIO 4     /* 低于舵机回读任务(5)，不抢 UART1 的读窗口 */
#define FPGA_RX_TASK_STACK 4096 /* 字节（ESP-IDF 的栈深度单位是字节） */

/* 控制台：用 UART0 + 独立行读取任务（不是 esp_console）。
 * 原因：本工程已经在用裸 printf，且没有 console 组件依赖；用一个只读
 * UART0 RX 的任务最小侵入，不装 REPL、不分配命令行对象。
 * ⚠️ sdkconfig 里 CONFIG_ESP_CONSOLE_UART_NUM 必须是 0（本工程就是），
 *    .c 里有编译期检查。 */
#define FPGA_CONSOLE_ENABLE 1
#define FPGA_CONSOLE_UART UART_NUM_0
#define FPGA_CONSOLE_RX_BUF 256
#define FPGA_CONSOLE_POLL_MS 50
#define FPGA_CONSOLE_LINE_MAX 48
#define FPGA_CONSOLE_TASK_PRIO 3
#define FPGA_CONSOLE_TASK_STACK 4096

/* ★ 2026-10-26 实机踩坑：控制台输入从哪儿读。
 *
 * 本板 sdkconfig 是 CONFIG_ESP_CONSOLE_UART_DEFAULT + SECONDARY_USB_SERIAL_JTAG，
 * printf 走 UART0(GPIO43/44)，而 USB 口是**原生 USB-Serial-JTAG**。
 * 板子上只接了 USB-JTAG，GPIO43/44 没有引出 —— 实测把 `fpga on` 打在 USB 口上，
 * UART0 的读任务一个字节都收不到，命令毫无反应（日志正常，因为 stdout 有次级镜像）。
 *
 * 所以默认改成从 usb_serial_jtag 驱动读输入（printf 本来就已从那儿出）。
 * ⚠️ 若以后另接一根 USB-TTL 到 GPIO43/44，把它改回 0 即可走 UART0。 */
#define FPGA_CONSOLE_VIA_USB_JTAG 1

/* ==========================================================================
 * 三、协议帧 / 超时
 * ========================================================================== */

#define FPGA_FRAME_LEN 35 /* 定长，含 \r\n；见 README 的逐字节表 */
#define FPGA_IMG_WIDTH 800  /* 协议 x 范围 0..799 */
#define FPGA_IMG_HEIGHT 480 /* 协议 y 范围 0..479 */

/* P/E 越界告警的最小间隔（避免坏帧刷屏；计数仍然每帧累加） */
#define FPGA_REJECT_WARN_MS 1000

/* 距最近一帧有效数据超过它 -> fpga_link_get() 返回 false。
 * 26.7 fps 的正常间隔约 37.5 ms；100 ms ≈ 丢 2.7 帧，足够区分“抖动”与“没了”。
 * 调用方拿 false 后自然回落到 ESP-NOW / HOLD，不在本模块里另造失联逻辑。 */
#define FPGA_STALE_MS 100

/* ==========================================================================
 * 四、P/E -> 通道值（单位换算是本模块最容易写错的地方）
 * ========================================================================== */

/* FPGA 侧视觉伺服帧率（实测）。P/E 的单位是“tick/帧”，不是 tick/s。 */
#define FPGA_TICK_FPS 26.7f

/* 下游满偏速度，必须与 rt_control.h 的 RC_RATE_FULL_TPS 一致。
 * 若以后改了下游速率，这里必须同步改；否则速度会整体偏。 */
#define FPGA_TPS_FULL 800.0f

/* ★★ 关键单位修正（2026-10 交接时确认）★★
 * FPGA 的 1 tick ≈ 0.5°；ESP32 舵机的 1 tick = 360°/4096 ≈ 0.0879°。
 * 两者不是同一个 tick！少了这个 5.689 倍系数，整套速度会慢 5.7 倍。
 *   1 FPGA tick = 0.5° = 0.5 / (360/4096) = 5.688888... 舵机 tick
 * 校核：P=4 -> q = 4×26.7×5.6889/800 = 0.7595 -> rate≈607 tick/s≈53°/s。 */
#define FPGA_TICK_TO_SERVO (4096.0f / 360.0f / 2.0f)

/* 三轴增益。先用 1.0/0.5/0.5；现场按“慢一点、别撞”的原则往小调。 */
#define FPGA_G_PAN 1.0f      /* CH1 底座 pan */
#define FPGA_G_SHOULDER 0.5f /* CH2 肩 reach */
#define FPGA_G_ELBOW 0.5f    /* CH3 肘 -reach（伸出去 = 肩肘反向） */

/* ==========================================================================
 * 五、安全限幅
 * ========================================================================== */

/* 单帧 P/E 绝对值上限。实测正常被 FPGA 限在 ±4；±5..±8 仍接受（留一点余量），
 * >8 视为解析错位/噪声，整帧丢弃，绝不让一个 999 打飞机械臂。 */
#define FPGA_INC_MAX 8

/* 软限位：模块自己累计的 pan/reach 位移（**单位是 ESP32 舵机 tick**，
 * 不是 FPGA tick！）。400 tick = 400×360/4096 ≈ 35.2°；
 * P=4 时约 607 tick/s，0.66 s 就会撞到这条虚拟边界。
 * 超过后本模块输出全 0 并锁存，必须控制台 `fpga reset` 才恢复。
 * 它是“开环视觉伺服跑飞”的最后一道保险，不是机械行程的替代品。 */
#define FPGA_SOFT_LIMIT_TICKS 1200.0f

/* ==========================================================================
 * 六、控制台透传显示
 * ========================================================================== */

/* 1 Hz 打印最近一帧。标定时用 PC 串口工具直接抄 R x,y。 */
#define FPGA_ECHO_PERIOD_MS 1000

/* 1 = 原样回显 33 字节可视载荷（去掉 \r\n，追加 age），
 * 0 = 用解析后的字段重新格式化（字段仍逐字一致）。
 * 两种都输出 `[FPGA] R... age=...ms`，不影响标定抄数。 */
#define FPGA_ECHO_RAW 1

/* ==========================================================================
 * 七、对外类型 / 接口
 * ========================================================================== */

/* 最近一帧解析结果（只读快照，不在中断里用） */
#include "vision_frame.h"

/* 控制台 `fpga stat` 用的统计快照 */
typedef struct {
    uint32_t rx_bytes;     /* UART2 收到的原始字节数（判断物理层通不通） */
    uint32_t frames_ok;    /* 严格校验通过且 P/E 未越界的帧数 */
    uint32_t frames_bad;   /* 长度/字符/坐标/行尾任一不符的坏帧数 */
    uint32_t inc_rejected; /* P/E 绝对值 > FPGA_INC_MAX 被丢弃的帧数 */
    uint32_t resyncs;      /* 帧中途又遇到 'R' 的次数（半帧/噪声证据） */
    uint32_t overflows;    /* 超过 35 字节仍未见到 \n 的次数 */
    uint32_t uart_errors;  /* UART 读失败 / TX 失败的次数 */
    uint32_t soft_limit_hits; /* 历史触发次数（reset 不清零，便于复盘） */
    bool enabled;          /* 当前是否允许输出 */
    bool ctrl_ready;       /* 当前是否有可输出的控制快照 */
    bool soft_limit_latched; /* 软限位是否锁存（true 时输出恒 0） */
    bool have_frame;       /* 是否至少收到过一帧有效数据 */
    float pan_travel;      /* 累计 pan 位移（舵机 tick，带符号） */
    float reach_travel;    /* 累计 reach 位移（舵机 tick，带符号） */
    int64_t last_frame_age_us; /* 距最近有效帧；无帧时为 -1 */
    fpga_frame_t last;     /* 最近有效帧 */
} fpga_link_stats_t;

/* 启动：安装 UART2、创建接收任务、安装 UART0 行读取任务。
 * 幂等；重复调用返回 ESP_OK。失败时已打印原因，应用可继续跑（不 abort）。
 * ⚠️ 必须在 app_main 里、进入 200 Hz 循环之前调用一次。 */
esp_err_t fpga_link_start(void);

/* 取当前控制帧。
 * true = 已使能 && 距最近有效控制帧 < FPGA_STALE_MS && 快照有效。
 * false = 未使能 / 超时 / 还没收到“使能之后”的帧；此时不改写 *out。
 *
 * ★ 注意：在 stale 窗口内它会**持续返回 true**（不是边沿）。这是调用方
 *   期望的速率语义：同一个 P/E 速度指令保持到下一帧到达。
 *   集成方式见 patch_rt_rx_main.md：true 时覆盖 last_ctrl，再把 fresh 置真。 */
bool fpga_link_get(rt_ctrl_t *out);

/* 手动使能/关闭。true 时清掉“控制已就绪”标志，必须等使能后的新帧才输出；
 * 不会清累计位移（要清用 fpga_link_reset_limits）。 */
void fpga_link_set_enable(bool enable);

/* ★ 2026-10-28：标定用的 test 模式 —— 只允许 `fpga jog` 输出，**不听 FPGA 帧**。
 * 用途：把某个关节单独推一个已知量/已知时长，量 d(半径)/d(关节)、d(角度)/d(关节)。
 * 进入 test 模式会自动关闭帧控制（s_enabled=0），退出用 `fpga off`。 */
void fpga_link_set_test(bool on);
bool fpga_link_is_enabled(void);

/* 清空 pan/reach 累计位移并解除软限位锁存。不会自动使能。 */
void fpga_link_reset_limits(void);

/* 最近是否收到过有效帧（与 enabled 无关）。
 * 位置式用它当"链路新鲜度"，这样**不需要遥控端在线**也能跟踪。 */
bool fpga_link_frame_fresh(void);

/* 复制最近一帧；无帧返回 false。 */
bool fpga_link_get_last(fpga_frame_t *out);

/* 复制统计快照；out 可为 NULL。 */
void fpga_link_get_stats(fpga_link_stats_t *out);

/* 控制台主机是否真的在收数据。USB-Serial-JTAG 控制台下主机断开时 printf
 * 会阻塞在 TX 里，把 200 Hz 主循环拖过 50 ms；没人读就不该打。 */
bool fpga_console_connected(void);

/* 预留：通过 UART2 TX 发 `C cx,cy\n`（注意是 \n，不是 \r\n）。
 * FPGA_PIN_TX < 0 时恒返回 false（默认不启用）；启用时把 FPGA_PIN_TX 改成
 * 实际 GPIO，并确保 TX 是 3.3 V 电平、共地、不与 UART1 的 GPIO17 冲突。
 * 不要在 200 Hz 控制循环里调用（最坏会等 TX FIFO 约 1 ms）。 */
bool fpga_link_send_cmd(int cx, int cy);

/* Single-slot console mailbox: only the main loop touches the level state. */
#include "rt_level.h"
bool fpga_link_take_level_request(rt_level_request_t *out);

#include "rt_height.h"
bool fpga_link_take_height_request(rt_height_request_t *out);

typedef struct { int action; float a,b,c,d,e; } vision_request_t;
bool fpga_link_take_vision_request(vision_request_t *out);
void fpga_link_auto_report(const char *text);
#endif /* FPGA_LINK_H */
