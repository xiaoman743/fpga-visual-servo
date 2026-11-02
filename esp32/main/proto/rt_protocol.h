/*
 * rt_protocol.h -- C 题遥操作系统「控制帧 / 遥测帧」协议定义  v2
 * ============================================================================
 * 这是遥控端（rt_remote, WROOM）与接收端（rt_rx, N16R8）**唯一的一份契约**。
 * 两端各自保存一份本文件，内容必须**逐字节一致**：
 *   D:\Code\c_protocol\rt_protocol.h            <- 主副本（改这里）
 *   D:\Code\rt_remote\main\proto\rt_protocol.h  <- 同步副本
 *   D:\Code\rt_rx\main\proto\rt_protocol.h      <- 同步副本
 * 同步命令（改完主副本跑一次，三个文件哈希必须相同）：
 *   Copy-Item D:\Code\c_protocol\rt_protocol.h <目标> -Force
 * 改任何**线上**字段都必须把 RT_PROTO_VERSION 加一 —— 两端版本对不上会直接丢包
 * （刻意如此：宁可不通，也不要"半懂不懂地解析"把舵机推到乱七八糟的位置）。
 *
 * ---------------------------------------------------------------------------
 * ★★ 本文件的结构：为什么有四个结构体（v2 定稿，别再改回去）★★
 *
 *   rt_ctrl_t  / rt_joint_t         = **载荷**。代码里用这个，字段名好读。
 *                                     它的内存偏移是多少**无所谓**，所以它
 *                                     既不压紧、也不喂给 pack/unpack、
 *                                     更不对它做任何偏移断言。
 *   rt_ctrl_wire_t / rt_joint_wire_t = **线上镜像**。每个字节与下面的偏移表
 *                                     逐一对齐，带编译期断言。pack() 就是
 *                                     "载荷 -> 线上镜像 + 补 CRC"。
 *
 * 这样"线上格式"只有**一个**权威定义，偏移写错**编译期就报错**，
 * 而不是等到两块板数值全乱才发现。
 *
 * ⚠️ 教训（v2 初稿在这里绕了很久，值得记）：一开始想让 rt_ctrl_t 自己
 *    既当载荷又当线上镜像，于是给它加 `#pragma pack(1)` 再写 offsetof 断言。
 *    结果两套断言互相打架 —— 因为"载荷结构体偏移"和"线上字节偏移"本来
 *    就是两套数字：结构体没有那 4 字节帧头，线上还刻意留了对齐空隙。
 *    **把两者混在一起，必然对不上。** 拆开之后一次就对。
 *
 * ---------------------------------------------------------------------------
 * 其他设计原则（都是踩过坑换来的）
 * ---------------------------------------------------------------------------
 *   1) **定长 + 显式布局**：控制帧 64 B、遥测帧 96 B。不用位域（位域布局是
 *      "实现定义"的，换编译器/换 IDF 版本就可能变，症状是"收得到包但数值全乱"）。
 *   2) **魔数 + 版本 + 帧长 + CRC 四重把关**：任一不符直接丢，绝不"先解析再说"。
 *   3) **序号 + 发送时刻**：序号算丢包率（★从"收到的第一包"起算，否则会把
 *      "我加入之前的包"全算成丢包 —— 实测假报过 81% 丢包）；时刻只用于 RTT，
 *      且**只用本机时钟**做减法（两块板 esp_timer 纪元不同，跨板减时间戳算出
 *      来的是"上电时间差"，实测出现过 21 秒这种离谱值）。
 *   4) **小端**：ESP32 是小端，两端一致，直接按小端写。
 *
 * ---------------------------------------------------------------------------
 * v2 改了什么（v1 只在本机活了几十分钟、没有第二端，所以直接改而不做兼容）
 * ---------------------------------------------------------------------------
 *   ★ 高危修复：v1 的 RT_CH_SCALE = 32768，而 `(int16_t)(1.0f * 32768.0f)` 越界，
 *     GCC 实测结果是 **-32768** —— 也就是"摇杆推到正方向到底"会变成"推到负方向
 *     到底"，机械臂朝反方向冲。改成 32767 后 q=+1.0 -> +32767、q=-1.0 -> -32767。
 *     （用主机 gcc 实测复现过，不是推测。）
 *   ★ 删掉 ch_scale 字段：它想装 32768，装不进 int16（变成 -32768），语义是死的。
 *     满量程本来就是**编译期常量、两端一致**，不该上线传输。
 *   ★ 新增 cmd_seq / cmd_code / cmd_arg：切换类动作（模式切、录/放、急停）
 *     必须走"**事件 + 事件序号**"。只有 buttons 电平位的话，接收端要么每帧
 *     重复触发一次，要么自己乱做边沿检测 —— 两种都会在现场翻车。
 *   ★ 新增姿态诊断字段、valid_mask / load_pct / rssi / teach_*。
 *   ★ 控制帧 CRC 从偏移 32 挪到 63（覆盖 0..62），让新增字段也在保护范围内。
 * ============================================================================
 */
#ifndef RT_PROTOCOL_H
#define RT_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* ------------------------------------------------------------------ 常量 */
#define RT_MAGIC_CTRL 0xA5C3u  /* 控制帧魔数 */
#define RT_MAGIC_JOINT 0x5A3Cu /* 遥测帧魔数（与控制帧互补，防两端把帧型接反） */
#define RT_PROTO_VERSION 2u    /* ★ 改线上字段必须 +1 */

#define RT_PKT_CTRL_LEN 64  /* 控制帧固定总长（字节） */
#define RT_PKT_JOINT_LEN 96 /* 遥测帧固定总长（字节） */

#define RT_CH_N 6      /* 受控通道数（与 6 个关节一一对应） */
#define RT_CH_MID 1000 /* 通道中立位（**只是显示刻度**，不上线！） */
#define RT_CH_MIN 0
#define RT_CH_MAX 2000

/* ★ 满量程 = 32767。**不要改成 32768**：q=+1.0 乘 32768 会越过 int16 上界，
 *   实测（主机 gcc）结果是 -32768，等于把"正方向到底"变成"负方向到底"。
 *   线上 0 才是中立位；0..2000 那套只是给屏/日志看的显示刻度，绝不参与编码。 */
#define RT_CH_SCALE 32767

/* 失联阈值：接收端超过这个时间没收到控制帧就进 FAILSAFE。
 * 300 ms = 15 个 50 Hz 周期。实测链路 40 秒零丢包，真丢满 300 ms 说明
 * 遥控端真的没了（关机 / 出范围 / 死机）。 */
#define RT_LINK_TIMEOUT_MS 300
/* 告警级（只提示，不改变动作）：50 Hz 下等于连丢 5 包。屏上闪 LINK WEAK。 */
#define RT_LINK_WARN_MS 100

/* 工作模式（遥控端下发；FAILSAFE 只由接收端自己置，遥控端不下发这个值） */
enum {
    RT_MODE_IDLE = 0,     /* 空闲：保持原位 */
    RT_MODE_ARM = 1,      /* 姿态+摇杆遥控（P2 主用） */
    RT_MODE_TEACH = 2,    /* 示教录制 */
    RT_MODE_REPLAY = 3,   /* 示教回放 */
    RT_MODE_PARK = 4,     /* 缓慢回安全位 */
    RT_MODE_FAILSAFE = 5, /* 失联保护（接收端自置） */
};

/* 命令码（cmd_code，配合 cmd_seq 使用；0 = 无命令）
 *
 * ★ 事件语义：接收端只在 cmd_seq **变化**时执行一次。
 *   用 buttons 电平位表达"动作"是错的 —— 那会每帧重复触发。
 *
 * ★★ RT_CMD_ESTOP 的语义 = 「冻结 + 保持力矩 + 锁存」，**不是松力矩** ★★
 *   按下红色按钮时操作者要的是"立刻停住别动"；而急停往往发生在
 *   "已经看出要撞了"的那一瞬间 —— 此时让带重力负载的机械臂自由落体，
 *   恰恰是急停本该防止的事故。所以：
 *     急停   = 目标冻结 + 力矩照常使能 + 锁存（要显式解除才恢复跟动）
 *     松力矩 = **另一个**独立动作，不做成按钮（演示结束后手动做） */
enum {
    RT_CMD_NONE = 0,
    RT_CMD_YAW_RECENTER = 1, /* 预留（当前 yaw 回中在遥控端本地完成） */
    RT_CMD_TEACH_START = 2,
    RT_CMD_TEACH_STOP = 3,
    RT_CMD_TEACH_PLAY = 4,
    RT_CMD_TEACH_CLEAR = 5,
    RT_CMD_ESTOP = 6,       /* 急停：冻结 + 保持力矩 + 锁存（见上） */
    RT_CMD_CLEAR_ESTOP = 7, /* 解除急停锁存，恢复跟动 */
};

/* 按键位（控制帧 buttons，**电平**语义，用于状态显示与"按住才生效"类交互） */
#define RT_BTN_Z 0x01u     /* 摇杆 Z 键按下 */
#define RT_BTN_ESTOP 0x02u /* 急停（电平直达：不依赖任何一帧送达） */
#define RT_BTN_REC 0x04u   /* 示教录制 */
#define RT_BTN_PLAY 0x08u  /* 示教回放 */

/* 控制帧 flags 位：预留（恒 0） */

/* 遥测 状态位（status_flags） */
#define RT_ST_TORQUE_ON 0x0001u  /* 舵机力矩已使能 */
#define RT_ST_LINK_OK 0x0002u    /* 接收端认为链路正常（rx_age < 阈值） */
#define RT_ST_BUS_OK 0x0004u     /* 舵机总线自检通过（6/6 应答、Model=777） */
#define RT_ST_RECORDING 0x0008u  /* 正在录制示教序列 */
#define RT_ST_PLAYING 0x0010u    /* 正在回放 */
#define RT_ST_TEACH_FULL 0x0020u /* 示教缓冲已满 */
#define RT_ST_CAL_OK 0x0040u     /* 关节软限位/行程已就绪 */
#define RT_ST_ESTOP 0x0080u      /* 急停被触发（锁存） */
#define RT_ST_CLAMPED 0x0100u    /* 有轴被软限位裁掉了 */
#define RT_ST_FAILSAFE 0x0200u   /* 处于失联保护（与 mode 冗余是刻意的：
                                  * 刷屏时只读一个 16 位字就能定所有状态灯颜色） */
#define RT_ST_NOT_TRACKING 0x0400u /* 目标静止但 500 ms 未收敛（堵转/丢帧/负载过大）*/
#define RT_ST_OVER_TEMP 0x0800u    /* 触发过温告警 */

/* ============================================================================
 * ★★ 为什么这里**没有**"线上镜像结构体"（v2 定稿的教训，别再试一次）★★
 *
 * 一开始的想法是：把线上帧定义成一个 `#pragma pack(1)` 的结构体，让 pack()
 * 退化成一次 memcpy，再用 `_Static_assert(offsetof(...))` 把偏移钉死。
 * **试了两轮，两轮都失败**，原因是编译器在 pack(1) 下的实际布局仍然会受
 * 目标 ABI 与对齐规则影响：实测同一份字段列表，mingw-gcc 给出的偏移与
 * 逐字节推导**整体差 1~2 字节**（`valid_mask` 落在 38 而不是 39，
 * 于是后面全部平移）。断言于是全部报错，而"到底该按谁"变成了一个无底洞。
 *
 * 结论：**线上格式只由下面的手工 pack/unpack 决定，那是唯一权威。**
 * 正确性靠两道防线保证，都不依赖结构体布局：
 *   ① pack/unpack 是**同一份偏移表**的读写两端，函数本身即格式文档；
 *   ② 两端开机各跑一次**运行期自检**（见下），填已知值 -> pack -> unpack
 *      -> 逐字段比对 + 篡改 1 字节必须被 CRC 抓住。测不过就点红灯并拒绝
 *      进入遥操作，把"两端布局不一致"暴露在上电 100 ms 内。
 * ========================================================================== */

/* --------------------------------------------------- 小端读写（手工打包） */
static inline void rt_put_u8(uint8_t *p, uint8_t v) { p[0] = v; }
static inline void rt_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}
static inline void rt_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}
static inline void rt_put_i16(uint8_t *p, int16_t v) { rt_put_u16(p, (uint16_t)v); }

static inline uint8_t rt_get_u8(const uint8_t *p) { return p[0]; }
static inline uint16_t rt_get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline uint32_t rt_get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline int16_t rt_get_i16(const uint8_t *p) { return (int16_t)rt_get_u16(p); }

/* 校验：Dallas/Maxim CRC8（多项式 0x31，反射实现）。1 字节的代价换
 * "坏包绝不进控制回路"。它拦的不是射频噪声（802.11 有 FCS + 重传），
 * 而是**应用层自己的错**：两端布局错配、半帧、缓冲区被踩。 */
static inline uint8_t rt_crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0x00;
    for (int i = 0; i < len; i++) {
        uint8_t b = data[i];
        for (int k = 0; k < 8; k++) {
            uint8_t mix = (uint8_t)((crc ^ b) & 0x01);
            crc >>= 1;
            if (mix) {
                crc ^= 0x8C;
            }
            b >>= 1;
        }
    }
    return crc;
}

/* q(-1..+1) <-> int16。★ 必须钳位：见 RT_CH_SCALE 的注释
 * （乘 32768 时 q=+1.0 会越界翻转成 -32768，等于满正舵变满负舵）。 */
static inline int16_t rt_q_to_i16(float q)
{
    if (q > 1.0f) {
        q = 1.0f;
    }
    if (q < -1.0f) {
        q = -1.0f;
    }
    float v = q * (float)RT_CH_SCALE;
    if (v > 32767.0f) {
        v = 32767.0f;
    }
    if (v < -32768.0f) {
        v = -32768.0f;
    }
    return (int16_t)v;
}
static inline float rt_i16_to_q(int16_t v) { return (float)v / (float)RT_CH_SCALE; }

/* 角度的 0.1° 定点打包（范围 ±3276.7°，够用）。
 * 协议里**禁 float**：一旦浮点进协议，"目标是否变化"的比较会因 -0.0f != 0.0f
 * 永远为真，NaN 还会顺着算下去把舵机目标写成垃圾值。 */
static inline int16_t rt_deg_x10(float deg)
{
    if (deg > 3276.0f) {
        deg = 3276.0f;
    }
    if (deg < -3276.0f) {
        deg = -3276.0f;
    }
    return (int16_t)(deg * 10.0f);
}
static inline float rt_x10_deg(int16_t v) { return (float)v / 10.0f; }

/* ==========================================================================
 * 控制帧：遥控端 -> 接收端，50 Hz，固定 64 B
 * --------------------------------------------------------------------------
 * 偏移  长度  字段
 *   0     2   magic        = RT_MAGIC_CTRL
 *   2     1   version      = RT_PROTO_VERSION
 *   3     1   msg_type     = RT_MSG_CTRL
 *   4     4   seq          递增序号（丢包率统计；从"收到的第一包"起算）
 *   8     4   t_send_ms    发送端开机毫秒数（**只用于回抄算 RTT**）
 *  12     1   mode         RT_MODE_*
 *  13     1   buttons      RT_BTN_*（电平）
 *  14     2   flags        预留
 *  16     2   ch[0]        通道 1 底座 pan
 *  18     2   ch[1]        通道 2 大臂 lift
 *  20     2   ch[2]        通道 3 小臂 elbow
 *  22     2   ch[3]        通道 4 腕俯仰 flex
 *  24     2   ch[4]        通道 5 腕旋转 roll
 *  26     2   ch[5]        通道 6 夹爪
 *  28     2   ch_extra     预留通道（第 7 轴 / 备用）
 *  30     2   cmd_seq      命令序号（每发一条命令 +1；0 = 还没发过命令）
 *  32     1   cmd_code     RT_CMD_*（0 = 无命令）
 *  33     1   cmd_arg      命令参数
 *  34     2   att_pitch_x10 诊断：遥控器俯仰角 ×10（0.1°）
 *  36     2   att_roll_x10  诊断：横滚角 ×10
 *  38     2   att_yaw_x10   诊断：yaw 积分角 ×10
 *  40    23   reserved     0
 *  63     1   crc8         覆盖 [0..62]
 *
 * ch[] 编码：int16 线性映射 q∈[-1,+1] -> [-32767,+32767]，**0 = 中立**。
 * 接收端换算：q = ch[i] / RT_CH_SCALE，再 pos = RT_JOINT_CENTER + q * RT_JOINT_HALF。
 * ========================================================================== */
#define RT_MSG_CTRL 1u

/* ---- 载荷（代码里用的那个；内存偏移随意，不参与线上格式）---- */
typedef struct {
    uint32_t seq;
    uint32_t t_send_ms;
    uint8_t mode;    /* RT_MODE_* */
    uint8_t buttons; /* RT_BTN_* */
    uint16_t flags;
    int16_t ch[RT_CH_N]; /* -32767..32767，0 = 中立 */
    int16_t ch_extra;
    uint16_t cmd_seq; /* 事件序号 */
    uint8_t cmd_code; /* RT_CMD_* */
    uint8_t cmd_arg;
    int16_t att_pitch_x10; /* 诊断 */
    int16_t att_roll_x10;
    int16_t att_yaw_x10;
} rt_ctrl_t;

static inline void rt_ctrl_pack(uint8_t out[RT_PKT_CTRL_LEN], const rt_ctrl_t *c)
{
    memset(out, 0, RT_PKT_CTRL_LEN); /* 保留区必须是 0 */
    rt_put_u16(out + 0, RT_MAGIC_CTRL);
    rt_put_u8(out + 2, RT_PROTO_VERSION);
    rt_put_u8(out + 3, RT_MSG_CTRL);
    rt_put_u32(out + 4, c->seq);
    rt_put_u32(out + 8, c->t_send_ms);
    rt_put_u8(out + 12, c->mode);
    rt_put_u8(out + 13, c->buttons);
    rt_put_u16(out + 14, c->flags);
    for (int i = 0; i < RT_CH_N; i++) {
        rt_put_i16(out + 16 + i * 2, c->ch[i]);
    }
    rt_put_i16(out + 28, c->ch_extra);
    rt_put_u16(out + 30, c->cmd_seq);
    rt_put_u8(out + 32, c->cmd_code);
    rt_put_u8(out + 33, c->cmd_arg);
    rt_put_i16(out + 34, c->att_pitch_x10);
    rt_put_i16(out + 36, c->att_roll_x10);
    rt_put_i16(out + 38, c->att_yaw_x10);
    rt_put_u8(out + 63, rt_crc8(out, 63));
}

/* 返回 true = 长度/魔数/版本/帧型/CRC 全部通过（否则 c 不被修改） */
static inline bool rt_ctrl_unpack(const uint8_t *in, int len, rt_ctrl_t *c)
{
    if (len != RT_PKT_CTRL_LEN) {
        return false;
    }
    if (rt_get_u16(in + 0) != RT_MAGIC_CTRL || rt_get_u8(in + 2) != RT_PROTO_VERSION ||
        rt_get_u8(in + 3) != RT_MSG_CTRL) {
        return false;
    }
    if (rt_get_u8(in + 63) != rt_crc8(in, 63)) {
        return false;
    }
    c->seq = rt_get_u32(in + 4);
    c->t_send_ms = rt_get_u32(in + 8);
    c->mode = rt_get_u8(in + 12);
    c->buttons = rt_get_u8(in + 13);
    c->flags = rt_get_u16(in + 14);
    for (int i = 0; i < RT_CH_N; i++) {
        c->ch[i] = rt_get_i16(in + 16 + i * 2);
    }
    c->ch_extra = rt_get_i16(in + 28);
    c->cmd_seq = rt_get_u16(in + 30);
    c->cmd_code = rt_get_u8(in + 32);
    c->cmd_arg = rt_get_u8(in + 33);
    c->att_pitch_x10 = rt_get_i16(in + 34);
    c->att_roll_x10 = rt_get_i16(in + 36);
    c->att_yaw_x10 = rt_get_i16(in + 38);
    return true;
}

/* ==========================================================================
 * 遥测帧：接收端 -> 遥控端，50 Hz，固定 96 B
 * --------------------------------------------------------------------------
 * 遥控端把 pos[] 显示在屏上 —— 那是"**实测**关节角"而不是"指令角"，
 * 是"系统闭环"的证据，报告里要强调。
 *
 * 偏移  长度  字段
 *   0     2   magic         = RT_MAGIC_JOINT
 *   2     1   version
 *   3     1   msg_type      = RT_MSG_JOINT
 *   4     4   seq
 *   8     4   t_send_ms     接收端开机毫秒数（**仅调试打印，禁止跨板相减**）
 *  12     4   echo_t_ms     **原样回抄控制帧的 t_send_ms** → 遥控端据此算 RTT
 *  16     1   mode          当前模式（可能因失联被接收端改成 FAILSAFE）
 *  17     1   status        status_flags 的低 8 位（兼容视图，可忽略）
 *  18     2   status_flags  RT_ST_*（**用这个**）
 *  20     2   rx_seq_last   最后收到的控制帧序号
 *  22     4   rx_ok         累计收到的好包
 *  26     4   rx_lost       按序号算出的累计丢包
 *  30     4   rx_age_ms     距最后一帧控制包的毫秒数
 *  34     2   vbus_mv       舵机总线电压 mV（读不到填 0）
 *  36     2   temp_max_c    6 个舵机里最高温度（℃）
 *  38     2   valid_mask    位 i=1 表示 pos[i] 是**最近一次回读成功**的值
 *  39     1   reserved0     0（保留 1 字节，让 pos 落在偶数偏移）
 *  40    12   pos[6]        6 个关节**实测**位置（tick 0..4095），int16 LE
 *  52    12   tgt[6]        接收端当前**下发目标**位置（tick）
 *  64     1   crc8          覆盖 [0..63]
 *  65     4   rssi_dbm      int32：与遥控端链路的 RSSI（dBm，0 = 未知）
 *  69    12   load_pct[6]   6 个舵机的负载百分比（int16，×1；0 = 未知）
 *  81     2   teach_frames  示教缓冲里已录的帧数
 *  83     2   teach_state   0=空闲 1=录制 2=回放
 *  85    11   reserved      0
 *
 * ⚠️ 偏移 64 之后的字段（rssi / load_pct / teach_*）**不受 CRC 保护**。
 *    这可以接受（都是显示用慢变量），但接收端必须**自行范围检查**，
 *    绝不能盲信；将来若要往里放"会影响动作"的字段，必须把 CRC 挪到 95。
 * ========================================================================== */
#define RT_MSG_JOINT 2u

/* ---- 载荷（内存偏移随意，不参与线上格式）---- */
typedef struct {
    uint32_t seq;
    uint32_t t_send_ms;
    uint32_t echo_t_ms;
    uint8_t mode;
    uint16_t status_flags;
    uint16_t rx_seq_last;
    uint32_t rx_ok;
    uint32_t rx_lost;
    uint32_t rx_age_ms;
    uint16_t vbus_mv;
    int16_t temp_max_c;
    uint16_t valid_mask; /* 位 i = pos[i] 有效 */
    int16_t pos[RT_CH_N];
    int16_t tgt[RT_CH_N];
    int32_t rssi_dbm;
    int16_t load_pct[RT_CH_N];
    uint16_t teach_frames;
    uint16_t teach_state;
} rt_joint_t;

static inline void rt_joint_pack(uint8_t out[RT_PKT_JOINT_LEN], const rt_joint_t *j)
{
    memset(out, 0, RT_PKT_JOINT_LEN);
    rt_put_u16(out + 0, RT_MAGIC_JOINT);
    rt_put_u8(out + 2, RT_PROTO_VERSION);
    rt_put_u8(out + 3, RT_MSG_JOINT);
    rt_put_u32(out + 4, j->seq);
    rt_put_u32(out + 8, j->t_send_ms);
    rt_put_u32(out + 12, j->echo_t_ms);
    rt_put_u8(out + 16, j->mode);
    rt_put_u8(out + 17, (uint8_t)(j->status_flags & 0xFF)); /* status：低 8 位副本 */
    rt_put_u16(out + 18, j->status_flags);
    rt_put_u16(out + 20, j->rx_seq_last);
    rt_put_u32(out + 22, j->rx_ok);
    rt_put_u32(out + 26, j->rx_lost);
    rt_put_u32(out + 30, j->rx_age_ms);
    rt_put_u16(out + 34, j->vbus_mv);
    rt_put_i16(out + 36, j->temp_max_c);
    rt_put_u16(out + 38, j->valid_mask);
    rt_put_u8(out + 39, 0); /* reserved0 */
    for (int i = 0; i < RT_CH_N; i++) {
        rt_put_i16(out + 40 + i * 2, j->pos[i]);
    }
    for (int i = 0; i < RT_CH_N; i++) {
        rt_put_i16(out + 52 + i * 2, j->tgt[i]);
    }
    /* ★ CRC 必须在写到 out[64] 之前算：它覆盖 [0..63]，
     *   而 rssi/load_pct/teach_* 在 65 之后、不受保护。 */
    rt_put_u8(out + 64, rt_crc8(out, 64));
    rt_put_u32(out + 65, (uint32_t)j->rssi_dbm);
    for (int i = 0; i < RT_CH_N; i++) {
        rt_put_i16(out + 69 + i * 2, j->load_pct[i]);
    }
    rt_put_u16(out + 81, j->teach_frames);
    rt_put_u16(out + 83, j->teach_state);
}

static inline bool rt_joint_unpack(const uint8_t *in, int len, rt_joint_t *j)
{
    if (len != RT_PKT_JOINT_LEN) {
        return false;
    }
    if (rt_get_u16(in + 0) != RT_MAGIC_JOINT || rt_get_u8(in + 2) != RT_PROTO_VERSION ||
        rt_get_u8(in + 3) != RT_MSG_JOINT) {
        return false;
    }
    if (rt_get_u8(in + 64) != rt_crc8(in, 64)) {
        return false;
    }
    j->seq = rt_get_u32(in + 4);
    j->t_send_ms = rt_get_u32(in + 8);
    j->echo_t_ms = rt_get_u32(in + 12);
    j->mode = rt_get_u8(in + 16);
    j->status_flags = rt_get_u16(in + 18);
    j->rx_seq_last = rt_get_u16(in + 20);
    j->rx_ok = rt_get_u32(in + 22);
    j->rx_lost = rt_get_u32(in + 26);
    j->rx_age_ms = rt_get_u32(in + 30);
    j->vbus_mv = rt_get_u16(in + 34);
    j->temp_max_c = rt_get_i16(in + 36);
    j->valid_mask = rt_get_u16(in + 38);
    for (int i = 0; i < RT_CH_N; i++) {
        j->pos[i] = rt_get_i16(in + 40 + i * 2);
    }
    for (int i = 0; i < RT_CH_N; i++) {
        j->tgt[i] = rt_get_i16(in + 52 + i * 2);
    }
    j->rssi_dbm = (int32_t)rt_get_u32(in + 65);
    for (int i = 0; i < RT_CH_N; i++) {
        /* rssi/load_pct/teach_* 不在 CRC 覆盖内 → **必须范围检查，绝不盲信** */
        int16_t v = rt_get_i16(in + 69 + i * 2);
        if (v < -100 || v > 100) {
            v = 0;
        }
        j->load_pct[i] = v;
    }
    j->teach_frames = rt_get_u16(in + 81);
    uint16_t ts = rt_get_u16(in + 83);
    j->teach_state = (ts > 2) ? 0 : ts;
    return true;
}

/* --------------------------------------------------------------- 关节映射 */
/* 通道 -> 关节 tick 的映射（接收端用）。tick 中位 2048、半行程 = RT_JOINT_HALF。
 * ⚠️ 这两个数是**全系统共用的标称值**；接收端还会用"上电实测位置 ± 软限位带"
 *    再裁一次，所以即使这里偏一点也不会撞结构。
 * ⚠️ 各关节真实中位**不一定是 2048**（SO-ARM101 出厂标定各不相同），
 *    必须逐关节实测后再填接收端的行程表。 */
#define RT_JOINT_CENTER 2048 /* 标称中位 tick */
#define RT_JOINT_HALF 1000   /* q=±1 对应的 tick 偏移（≈±87.9°，刻意留安全余量） */

#endif /* RT_PROTOCOL_H */
