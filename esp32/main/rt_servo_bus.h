/*
 * rt_servo_bus.h -- STS3215 总线舵机驱动（SCS 协议）接口
 * 实测结论与接线见 rt_servo_bus.c 顶部；这里只放接口与寄存器表。
 */
#ifndef RT_SERVO_BUS_H
#define RT_SERVO_BUS_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/uart.h"

/* ---------------------------------------------------------------- 硬件接线
 * ⚠️ 这是**唯一正确的一版**（2026-10-01 实测 6/6）：
 *   板 TTL 排针 `TX` → GPIO18(RX) ；板 `RX` → 【983 Ω】→ GPIO17(TX) ；GND↔GND
 *   ⛔ 绝不接 `A/B/V2/G`（RS485 差分侧，接上去永远 0 字节） */
#define SB_UART UART_NUM_1
#define SB_PIN_TX 17
#define SB_PIN_RX 18
#define SB_BAUD 1000000 /* SO-ARM101 的 STS3215 出厂 1,000,000 */
#define SB_RX_BUF 2048
#define SB_READ_TIMEOUT_MS 25 /* 与探针固件一致：实测偶发丢帧就靠它兜 */
#define SB_INTERFRAME_US 200   /* 帧间空隙：太小会让舵机来不及取走命令 */
#define SB_SYNC_MAX 8          /* 同步写一帧最多几轴（实际用 6） */

/* ---------------------------------------------------------------- 读超时
 * ⚠️ 这两个数直接决定控制循环能不能守住 200 Hz，别随便调大：
 *   放进 200 Hz 回路的读超时必须是**毫秒级**（探针固件用 25 ms 是为了
 *   一次性诊断，放进控制回路会把周期整个拖垮 —— 实测掉到 46 Hz）。
 *   实测案例（2026-10-02 晚）：扳动机械臂之后线束被拉动，总线开始丢帧，
 *   rdfail 涨到 32~45‰，行首频率掉到 46/105/113 Hz，现场表现就是
 *   "推了没反应、臂一顿一顿"。⇒ 见下面 SB_BAUD 的降速办法。 */
#define SB_READ_TIMEOUT_SHORT_MS 6 /* 200 Hz 回路里用这个（见 sbus_read_status） */

/* ---------------------------------------------------------------- 波特率
 * 1 Mbps 是出厂值，也是**实测 6/6 + rdfail=0** 通了的值。
 * ⚠️ 但如果出现"大量 rdfail / 控制循环掉频 / 偶发丢帧"，第一嫌疑人不是代码，
 *    而是**信号完整性**：
 *      · 板上 TX 线上串的 983 Ω 电阻 + 线束电容 → 边沿变慢；
 *      · 机械臂一动，线束被拉动/重新走线 → 线长与耦合都变了。
 *    此时把 SB_BAUD 改成 500000（或 250000）重烧，往往立刻恢复。
 *    —— 文档 SOP §3.4 ④ 记的就是这条，别再从代码查起。 */

/* 是否使用同步写（一条广播帧更新全部轴）。0 = 退回逐轴写（排故用）。
 * ⚠️ 逐轴写在 200 Hz 下**必然超时**（6 帧 ≈ 3 ms + 6 次应答），
 *    只适合把频率降到 50 Hz 时排故。 */
#define SB_SYNC_WRITE_ENABLE 1

/* ---------------------------------------------------------- STS3215 寄存器
 * 与 D:\Code\urt2_probe_idf 的探针固件**逐条一致**（那份已实测跑通，别自己猜） */
#define SB_REG_MODEL 3            /* 型号：STS3215 = 777 */
#define SB_REG_ID 5               /* 舵机 ID（⚠️ 别乱写，会改 ID） */
#define SB_REG_BAUD 6             /* 波特率（⚠️ 别乱写） */
#define SB_REG_MIN_ANGLE 9        /* 最小限位 */
#define SB_REG_MAX_ANGLE 11       /* 最大限位 */
#define SB_REG_MAX_TORQUE 16      /* 最大力矩 */
#define SB_REG_TORQUE_ENABLE 40   /* 力矩开关 0/1 */
#define SB_REG_ACCELERATION 41    /* 加速度（同步写的起始地址就是它） */
#define SB_REG_GOAL_POSITION 42   /* 目标位置（2 字节） */
#define SB_REG_GOAL_SPEED 46      /* 运行速度（2 字节） */
#define SB_REG_TORQUE_LIMIT 48    /* 力矩限制 */
#define SB_REG_LOCK 55            /* 锁（写 EEPROM 前要先解锁） */
#define SB_REG_PRESENT_POSITION 56 /* **实测位置**（2 字节） */
#define SB_REG_PRESENT_SPEED 58
#define SB_REG_PRESENT_LOAD 60
#define SB_REG_PRESENT_VOLTAGE 62 /* 单位 0.1 V */
#define SB_REG_PRESENT_TEMPERATURE 63
#define SB_REG_MOVING 66

#define SB_STS3215_MODEL 777

/* SCS 指令码 */
#define SCS_INST_PING 0x01
#define SCS_INST_READ 0x02
#define SCS_INST_WRITE 0x03
#define SCS_INST_SYNC_WRITE 0x83
#define SCS_BROADCAST_ID 0xFE

/* ------------------------------------------------------------------ 接口 */
void sbus_init(void);
bool sbus_is_inited(void);

/* 总线互斥：一次事务（请求+应答）期间不能被别的任务插队。
 * ⚠️ 读任务和控制循环都会用总线 —— **不加锁就会出现"回帧串台"**：
 *    A 任务的请求收到了 B 任务的应答，值看起来"合理"所以极难发现。 */
void sbus_lock(void);
void sbus_unlock(void);

bool sbus_ping(uint8_t id, uint8_t *err);
bool sbus_read_reg(uint8_t id, uint8_t addr, uint8_t len, uint8_t *out);
bool sbus_write_reg(uint8_t id, uint8_t addr, const uint8_t *data, int len);
bool sbus_read_u16(uint8_t id, uint8_t addr, uint16_t *out);
bool sbus_read_u8(uint8_t id, uint8_t addr, uint8_t *out);
bool sbus_write_u16(uint8_t id, uint8_t addr, uint16_t v);
bool sbus_write_u8(uint8_t id, uint8_t addr, uint8_t v);

/* 读一个轴的状态块：**一条 READ(56, 8) 拿到位置+速度+负载+电压+温度**。
 * 寄存器 56..63 是连续的（Present_Position=56/2B, Speed=58/2B, Load=60/2B,
 * Voltage=62/1B, Temperature=63/1B），所以一次事务就够 —— 原来分 3 次读，
 * 事务数和最坏耗时都是 3 倍，正是控制循环掉频的帮凶。
 * 传 NULL 表示这个量不要（仍然只发一条帧，只是不解那个字段）。 */
bool sbus_read_status(uint8_t servo_id, uint16_t *pos, uint16_t *speed, uint16_t *load,
                      uint8_t *volt, uint8_t *temp, int timeout_ms);

/* 6 个轴一起开/关力矩。关力矩前必须确认臂已被托住 —— 否则会因重力掉下来 */
void sbus_torque_all(const uint8_t *ids, int n, bool on);

/* 同步写：一帧广播更新 n 个轴的目标位置（acc=加速度，speed=运行速度） */
bool sbus_sync_write_goal(const uint8_t *ids, int n, const uint16_t *pos, uint8_t acc,
                          uint16_t speed);

/* 诊断：取走"本窗口收到的原始字节数"（取完清零）+ 累计超时次数。
 * ★ 用途：区分"**一个字节都没收到**"（物理层：线/UART 配置）与
 *   "**收到字节但解析不出**"（协议/波特率）—— 这两种故障的排查方向完全相反，
 *   只看 NO ANSWER 分不清。2026-10-02 实测为此白查了很久，所以固化成接口。 */
uint32_t sbus_raw_bytes_take(void);
uint32_t sbus_txn_timeouts(void);
/* ★ 2026-10-11：回帧 ID 失配（迟到应答窜窗）拒收计数。持续上涨 = 总线时序
 * 临界（信号完整性/波特率），但遥测不再被假位置污染。 */
uint32_t sbus_id_mismatches(void);
void sbus_raw_hex_dump(void);

#endif /* RT_SERVO_BUS_H */
