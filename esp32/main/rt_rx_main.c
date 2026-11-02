/*
 * rt_rx_main.c -- C 题接收端（机械臂侧）固件  v1
 * ============================================================================
 * 这是 P2 的核心：**从零写的接收端控制固件**（原来板上跑的是 espnow_link
 * 链路验证固件，不是控制固件）。
 *
 * 数据流（单向一环）：
 *   遥控端 --(ESP-NOW 控制帧 64B, 50Hz)--> 本机
 *      -> 6 通道 -> 关节目标 tick（含软限位裁剪）
 *      -> 200 Hz 控制律：插值 + 加速度限制 + 限速 + 死区（rt_control.c）
 *      -> **一帧同步写（SCS SYNC WRITE, ID=0xFE）** 更新 6 个 STS3215
 *   本机 --(ESP-NOW 遥测帧 96B, 50Hz)--> 遥控端（回传**实测**关节角，证明闭环）
 *
 * ★ 安全设计（这些都是"演示现场不翻车"的关键，别删）：
 *   1) **上电先读位置再开力矩**：直接把目标设成 2048 会让臂"啪"地跑到中间，
 *      可能撞桌子/撞限位。正确做法是把目标初始化成**当前实际位置**。
 *   2) **失联不松力矩**：松了力矩，6 个关节会因重力直接掉下来（很危险）。
 *      失联策略 = 冻结目标 + 保持力矩 = 顶住当前姿态。这是题目基础第 3 条
 *      「载荷不下滑」在失效路径上的对应做法。
 *   3) **总线不通过不开力矩**：6/6 不应答就只打印诊断、绝不使能，
 *      免得只驱动了一半关节（那会扭坏结构）。
 *   4) 电压/温度超限只**告警+回传**，不自动松力矩（自动松力矩反而更危险）。
 *
 * 接线（与已验证的探针工程一致，⛔ 不是 A/B/V2/G 那套 RS485）：
 *   ESP32-S3 N16R8（MAC 7C:E8:B1:A3:E4:70，COM9）
 *     板 TTL `TX` → GPIO18(RX)
 *     板 TTL `RX` → 【983 Ω】 → GPIO17(TX)
 *     GND ↔ GND ；板上 `5V` / `DTR` 都不接
 *   接收端 ESP32 **独立供电**（绝不从舵机那路 5V 取电 —— 6 舵机换向会把母线拉塌）
 *
 * 输出全 ASCII，任何终端都不乱码。
 * !! 本文件只允许用编辑工具修改（shell 文本工具会按 GBK 解码毁掉中文注释）。
 * ============================================================================
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "rt_control.h"
#include "rt_protocol.h"
#include "rt_servo_bus.h"
#include "fpga_link.h"
#include "fpga_pos.h"
#include "vision_pick.h"
#include "vision_grip.h"
#include "operator_keys.h"
#include "vision_auto.h"
#include "vision_reach.h"
#include "vision_probe.h"
#include "vision_calibrate.h"
#include "boot_profile.h"
#include "rt_level.h"
#include "rt_height.h"

/* ============================== 配置 ============================== */
#define ESPNOW_CHANNEL 6 /* 必须与遥控端一致（实测值） */

/* 6 个舵机的 ID：SO-ARM101 出厂就是 1..6，与 LeRobot 一致 */
static const uint8_t SERVO_ID[RC_NJ] = {1, 2, 3, 4, 5, 6};

/* 上电后等多久再开始读舵机（给自己留出打开监视器/架好臂的时间）。
 * 反正是只读阶段，不会动臂。 */
#define BOOT_DELAY_MS 2000

/* 失联阈值：超过它没收到控制帧 → FAILSAFE（冻结目标 + 保持力矩）。
 * 300 ms = 15 个 50 Hz 周期。实测链路 40 秒零丢包，真丢这么久 = 遥控端真没了。 */
#define LINK_TIMEOUT_MS RT_LINK_TIMEOUT_MS

/* 同步写的运行速度/加速度。speed 是舵机内部限速（我们外层也限了），
 * 留 0 = 用舵机自己的最大速度，由我们的插值决定快慢。 */
#define GOAL_SPEED 0
#define GOAL_ACC 20 /* 加速度档位（0-254），20 比较温和 */

/* 软限位带宽：以**上电时的实际位置**为中心，左右各 ±RC_TRAVEL tick。
 *
 * ⚠️ 但这个值只是"拍脑袋的安全带"，**不是**真正的机械边界。真正的边界有两个来源：
 *   ① 舵机 EEPROM 里的 Min/Max_Position_Limit（出厂可能没设 → 0..4095 = 无效）
 *   ② 结构本身的物理极限（撞上去就是关节4/关节5 那种"转死了"）
 * 所以最终软限位 = **(上电位置 ± RC_TRAVEL) ∩ (EEPROM 限位 ± RC_MARGIN)**，
 * 见 bus_bringup() 里的计算。2026-10-02 晚实测教训：
 *   关节4 位置 886 而 EEPROM limit_min=875 → **只剩 11 tick**，
 *   而我的 ±1200 当时还"以为"很安全 —— 拍脑袋的带宽根本挡不住真实限位。 */
#define RC_TRAVEL 1200

/* 离 EEPROM 限位至少留这么多 tick。撞上物理限位会丢步、会让 LeRobot 标定漂，
 * 所以宁可少走一点，也不要贴着边界跑。 */
#define RC_MARGIN 60

/* ---------------------------------------------------------------- 堵转保护
 * 判据三条件同时满足才动手（见主循环里的实现）：
 *   ① 目标与实测差 > RC_STALL_TICKS
 *   ② 持续 > RC_STALL_MS
 *   ③ 期间实测几乎没动（< RC_STALL_MOVE tick）
 * 现场演示时这一条是保命的：撞死角的舵机几十秒就能烫坏。 */
#define RC_STALL_TICKS 180 /* 差这么多才算"追不上"（正常跟随远小于它） */
#define RC_STALL_MS 1500   /* 持续这么久 */
#define RC_STALL_MOVE 25   /* 期间实测位移小于它 = 没在动 */
static rc_stall_guard_t s_stall_guard[RC_NJ];
static bool s_stall_latched[RC_NJ];

/* 回读任务：每 READ_PERIOD_MS 读一个轴（轮询），所以每轴刷新 = 6 * 周期。
 * ⚠️ READ_PERIOD_MS 直接决定总线占用：6 轴 × 10 ms = 每轴 60 ms 刷新一次。
 *    想更快就调小，但要注意"读"和 200 Hz 的"同步写"抢同一个 UART。 */
#define READ_PERIOD_MS 6
#define READ_TASK_STACK 4096

/* 回读健康度滑窗：用来判断"舵机电源是不是掉了"（见 read_task 里的 [PWR] 告警） */
#define RD_HIST_N 200
static uint8_t s_rd_hist[RD_HIST_N];
static int s_rd_hist_i = 0;
static volatile bool s_rail_warned = false;
/* 舵机电源是否健康（最近一窗回读成功率过半）。**堵转保护必须先看这一位**：
 * 断电时位置读数是垃圾值，拿它判堵转会把好关节的力矩误松掉（实测踩过）。 */
static volatile bool s_rail_ok = false;

/* ============================== 全局状态 ============================== */
static rc_joint_t s_j[RC_NJ];
static rt_level_t s_level;
static rt_height_t s_height;
static vision_pick_t s_pick;
static vision_grip_t s_grip_seek;
static bool s_adaptive_grip;
static int64_t s_grip_seek_read_us;
static operator_keys_t s_operator_keys;
static uint32_t s_operator_seq;
static bool s_service_hold,s_service_open,s_key_restart;
static float s_service_pose[RC_NJ];
static vision_auto_t s_auto;
static vision_probe_t s_probe;
static vision_calibrate_t s_calibrate;
static char s_calibration_error[192];
static char s_torque_error[128];
static void torque_failure(int axis,const char *stage) {
    if(!s_torque_error[0])snprintf(s_torque_error,sizeof(s_torque_error),
        "torque switch failed: CH%d %s",axis,stage);
}
static bool s_boot_profile_pending=true;
static unsigned s_boot_healthy_ms;
static unsigned s_boot_prepare_ms;
static bool s_margin_recover_logged;
static int16_t s_auto_motion_ref[4];
static float s_auto_motion;
static uint32_t s_visual_seq;
static bool s_visual_closed;
static bool s_position_was_selected;
static bool s_level_limits_ok;
static float s_level_lo[3], s_level_hi[3];
static volatile uint8_t s_joint_temp[RC_NJ];
static volatile int64_t s_joint_read_us[RC_NJ];
static volatile bool s_torque_on = false;
static bool s_boot_torque_released;
static volatile bool s_bus_ok = false;
static volatile bool s_cal_ok = false;

/* 最近一帧控制（WiFi 任务写 / 主循环读；用"快照+标志"避免撕裂） */
static volatile bool s_ctrl_fresh = false;
static rt_ctrl_t s_ctrl_shadow;
static volatile int64_t s_ctrl_last_us = 0; /* 本机时钟 */
static volatile uint8_t s_mode = RT_MODE_IDLE;

/* 接收统计 */
static volatile uint32_t s_rx_ok, s_rx_bad, s_rx_lost;
static volatile int32_t s_rx_last_seq = -1;
static volatile uint32_t s_tx_joint;
static volatile bool s_peer_ready = false;
static uint8_t s_peer_mac[6];
static volatile bool s_first_recv_logged = false;

/* 舵机总线健康 */
static volatile uint16_t s_vbus_mv;
static volatile int16_t s_temp_max;
static volatile uint32_t s_read_ok, s_read_fail;

/* 命令事件（cmd_seq/cmd_code）：只在序号变化时执行一次。
 * ★ 急停锁存：急停 = 冻结 + 保持力矩 + 锁存，**不是松力矩**（见 rt_protocol.h）。 */
static volatile bool s_estop_latched = false;
static uint16_t s_last_cmd_seq = 0;
static bool s_cmd_seq_init = false;
/* 软限位是否正在裁掉目标（遥测 RT_ST_CLAMPED 用；send_telemetry 在别的函数里，
 * 所以必须用文件级变量而不是主循环里的局部变量） */
static volatile bool s_clamped_g = false;

/* ============================== 小工具 ============================== */
static void mac_str(const uint8_t *m, char *out)
{
    sprintf(out, "%02X:%02X:%02X:%02X:%02X:%02X", (unsigned)m[0], (unsigned)m[1], (unsigned)m[2],
            (unsigned)m[3], (unsigned)m[4], (unsigned)m[5]);
}

static void print_banner(void)
{
    printf("\n######################################################\n");
    printf("#  RT RX  v1  -  C-problem P2 receiver (6x STS3215)  #\n");
    printf("######################################################\n");
    printf("SERVO BUS : UART%d TX=GPIO%d (via 983R)  RX=GPIO%d  BAUD=%d\n", (int)SB_UART, SB_PIN_TX,
           SB_PIN_RX, SB_BAUD);
    printf("PROTO     : ctrl=%d B joint=%d B ver=%d | CH%d mid=%d range=%d..%d\n",
           RT_PKT_CTRL_LEN, RT_PKT_JOINT_LEN, RT_PROTO_VERSION, RT_CH_N, RT_CH_MID, RT_CH_MIN,
           RT_CH_MAX);
    printf("CONTROL   : %d Hz period=%d ms | joint center=%d half=%d | vmax=6.0 tick/cyc\n",
           1000 / RC_PERIOD_MS, RC_PERIOD_MS, RT_JOINT_CENTER, RT_JOINT_HALF);
    printf("SAFETY    : link timeout=%d ms -> hold position (torque stays ON)\n",
           LINK_TIMEOUT_MS);
    printf("!! BEFORE POWER: arm must have free space to move; 5V 4A ON.\n");
}

/* 协议自检：与遥控端那份等价。两端都跑一次，排除"两端布局不一致"。
 * ⚠️ 必须**逐字段**比对，不能 memcmp 整个结构体 —— 结构体里的填充字节
 *    不参与线上传输，解包侧保持原值，memcmp 必然报失败（踩过）。 */
static bool proto_selftest(void)
{
    bool all_ok = true;
    rt_ctrl_t c0, c1;
    memset(&c0, 0, sizeof(c0));
    memset(&c1, 0, sizeof(c1));
    c0.seq = 0x12345678u;
    c0.t_send_ms = 0x0BADF00Du;
    c0.mode = RT_MODE_ARM;
    c0.buttons = RT_BTN_Z;
    c0.flags = 0x0005;
    for (int i = 0; i < RT_CH_N; i++) {
        c0.ch[i] = (int16_t)(-32767 + i * 1000);
    }
    c0.ch_extra = -1;
    c0.cmd_seq = 3;
    c0.cmd_code = RT_CMD_YAW_RECENTER;
    c0.cmd_arg = 7;
    c0.att_pitch_x10 = -123;
    c0.att_roll_x10 = 456;
    c0.att_yaw_x10 = 3276;
    uint8_t buf[RT_PKT_CTRL_LEN];
    rt_ctrl_pack(buf, &c0);
    bool ok = rt_ctrl_unpack(buf, sizeof(buf), &c1);
    if (ok) {
        ok = c1.seq == c0.seq && c1.t_send_ms == c0.t_send_ms && c1.mode == c0.mode &&
             c1.buttons == c0.buttons && c1.flags == c0.flags && c1.ch_extra == c0.ch_extra &&
             c1.cmd_seq == c0.cmd_seq && c1.cmd_code == c0.cmd_code && c1.cmd_arg == c0.cmd_arg &&
             c1.att_pitch_x10 == c0.att_pitch_x10 && c1.att_roll_x10 == c0.att_roll_x10 &&
             c1.att_yaw_x10 == c0.att_yaw_x10;
        for (int i = 0; i < RT_CH_N; i++) {
            if (c1.ch[i] != c0.ch[i]) {
                ok = false;
            }
        }
    }
    if (!ok) {
        printf("[PROTO] *** SELF-TEST FAILED (ctrl) -- the two ends will NOT agree ***\n");
        all_ok = false;
    } else {
        printf("[PROTO] ctrl self-test OK (ver=%d, %d B, field-by-field)\n", RT_PROTO_VERSION,
               RT_PKT_CTRL_LEN);
    }
    buf[20] ^= 0x01;
    if (rt_ctrl_unpack(buf, sizeof(buf), &c1)) {
        printf("[PROTO] *** tamper NOT detected (BAD!) ***\n");
        all_ok = false;
    } else {
        printf("[PROTO] ctrl tamper detected OK\n");
    }

    rt_joint_t j0, j1;
    memset(&j0, 0, sizeof(j0));
    memset(&j1, 0, sizeof(j1));
    j0.seq = 7;
    j0.t_send_ms = 1234;
    j0.echo_t_ms = 0xDEADBEEFu;
    j0.mode = RT_MODE_ARM;
    j0.status_flags = RT_ST_TORQUE_ON | RT_ST_BUS_OK;
    j0.rx_seq_last = 99;
    j0.rx_ok = 1000;
    j0.rx_lost = 2;
    j0.rx_age_ms = 17;
    j0.vbus_mv = 6600;
    j0.temp_max_c = 31;
    j0.valid_mask = 0x3F;
    j0.rssi_dbm = -57;
    for (int i = 0; i < RT_CH_N; i++) {
        j0.pos[i] = (int16_t)(2000 + i);
        j0.tgt[i] = (int16_t)(2100 + i);
        j0.load_pct[i] = (int16_t)(i * 3);
    }
    j0.teach_frames = 240;
    j0.teach_state = 1;
    uint8_t jb[RT_PKT_JOINT_LEN];
    rt_joint_pack(jb, &j0);
    bool jok = rt_joint_unpack(jb, sizeof(jb), &j1);
    if (jok) {
        jok = j1.seq == j0.seq && j1.t_send_ms == j0.t_send_ms && j1.echo_t_ms == j0.echo_t_ms &&
              j1.mode == j0.mode && j1.status_flags == j0.status_flags &&
              j1.rx_seq_last == j0.rx_seq_last && j1.rx_ok == j0.rx_ok && j1.rx_lost == j0.rx_lost &&
              j1.rx_age_ms == j0.rx_age_ms && j1.vbus_mv == j0.vbus_mv &&
              j1.temp_max_c == j0.temp_max_c && j1.valid_mask == j0.valid_mask &&
              j1.rssi_dbm == j0.rssi_dbm && j1.teach_frames == j0.teach_frames &&
              j1.teach_state == j0.teach_state;
        for (int i = 0; i < RT_CH_N; i++) {
            if (j1.pos[i] != j0.pos[i] || j1.tgt[i] != j0.tgt[i] ||
                j1.load_pct[i] != j0.load_pct[i]) {
                jok = false;
            }
        }
    }
    if (!jok) {
        printf("[PROTO] *** SELF-TEST FAILED (joint) ***\n");
        all_ok = false;
    } else {
        printf("[PROTO] joint self-test OK (%d B, field-by-field)\n", RT_PKT_JOINT_LEN);
    }
    return all_ok;
}

/* ============================== ESP-NOW ============================== */
static void rx_on_send(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    (void)status;
}

static void rx_on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    rt_ctrl_t c;
    if (!rt_ctrl_unpack(data, len, &c)) {
        s_rx_bad++;
        return;
    }
    /* 丢包统计：★ 从我收到的第一包起算（否则会把我加入之前的包全算成丢包，
     * 实测假报过 81% 丢包） */
    if (s_rx_last_seq < 0) {
        s_rx_last_seq = (int32_t)c.seq;
    } else if ((int32_t)c.seq > s_rx_last_seq) {
        uint32_t gap = (uint32_t)((int32_t)c.seq - s_rx_last_seq - 1);
        if (gap < 200) { /* 序号大跳 = 对方重启了，不算丢包 */
            s_rx_lost += gap;
        }
        s_rx_last_seq = (int32_t)c.seq;
    }
    s_rx_ok++;
    memcpy((void *)&s_ctrl_shadow, &c, sizeof(c));
    s_ctrl_fresh = true;
    s_ctrl_last_us = esp_timer_get_time();
    s_mode = c.mode;

    /* 记住对方 MAC 并升级成单播 peer（单播有 ACK、更省信道）。
     * ⚠️ 加 peer 只需成功一次，之后重复加会返回 ESP_ERR_ESPNOW_EXIST（无害）。
     * 📌 小坑：esp_now_recv_info_t 里是 `uint8_t *src_addr`（**非 const**），
     *    而本工程的其他接口都用 const uint8_t* —— 直接传会撞
     *    `-Werror=pointer-sign`。所以一律先 memcpy 到自己的缓冲再传。 */
    uint8_t from[6];
    memcpy(from, info->src_addr, 6);
    if (memcmp(s_peer_mac, from, 6) != 0) {
        memcpy(s_peer_mac, from, 6);
        esp_now_peer_info_t peer;
        memset(&peer, 0, sizeof(peer));
        memcpy(peer.peer_addr, from, 6);
        peer.channel = ESPNOW_CHANNEL;
        peer.ifidx = WIFI_IF_STA;
        peer.encrypt = false;
        esp_err_t e = esp_now_add_peer(&peer);
        s_peer_ready = true;
        char m[20];
        mac_str(from, m);
        printf("\n[NOW] *** control frames from %s (add_peer=%s) ***\n", m, esp_err_to_name(e));
    }
    if (!s_first_recv_logged) {
        s_first_recv_logged = true;
        printf("[NOW] first control frame: seq=%lu mode=%d ch=%d %d %d %d %d %d\n",
               (unsigned long)c.seq, c.mode, c.ch[0], c.ch[1], c.ch[2], c.ch[3], c.ch[4], c.ch[5]);
    }
    /* ★ 前 20 帧原始通道值全打出来。
     * 用途：万一"臂自己往死角转"，这一串能立刻回答**是哪一路通道在下指令** ——
     * 全 0 却还在动 → 问题不在控制指令（舵机零点/机械）；
     * 某一路非 0 → 直接锁定是遥控端那一通道的问题。
     * 这一条是"先怀疑方法"的落地：不要把"谁在下指令"留给猜。 */
    if (s_rx_ok < 20) {
        printf("[NOW] fw#%lu ch=%6d %6d %6d %6d %6d %6d\n", (unsigned long)s_rx_ok, c.ch[0],
               c.ch[1], c.ch[2], c.ch[3], c.ch[4], c.ch[5]);
    }
}

static void wifi_espnow_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    /* ★ 省电必须关：modem sleep 会让 ESP-NOW 出现周期性收包失败
     *   （现象是"每几包丢一包"，最容易被误判成链路质量差或代码 bug） */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(rx_on_send));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(rx_on_recv));

    /* 加一个广播 peer：这样**遥控端不需要知道我们的 MAC**（换板子不用重烧）。
     * 回传遥测走单播（收到控制帧后自动 add_peer 对方 MAC）。 */
    const uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, bcast, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
    char m[20];
    mac_str(mac, m);
    printf("[NOW] receiver MAC = %s  channel=%d  ps=NONE\n", m, ESPNOW_CHANNEL);
}

/* ============================== 舵机总线初始化 ============================== */
/* 扫描 + 读初始位置。**只读**，绝对不写任何寄存器（尤其不写 ID/波特率）。 */
static bool bus_bringup(void)
{
    int found = 0;
    uint16_t start_pos[RC_NJ], hot_goal[RC_NJ];
    bool hot_valid[RC_NJ]={0},torque_released[RC_NJ]={0};
    uint16_t lim_min[RC_NJ], lim_max[RC_NJ];
    bool lim_ok[RC_NJ];
    for (int i = 0; i < RC_NJ; i++) {
        start_pos[i] = RT_JOINT_CENTER; /* 先给个安全缺省 */
        lim_min[i] = 0;
        lim_max[i] = 4095;
        lim_ok[i] = false;
    }

    for (int attempt = 0; attempt < 2; attempt++) {
        found = 0;
        printf("\n[BUS] scan ID 1..6 @ %d baud (attempt %d)\n", SB_BAUD, attempt + 1);
        for (int i = 0; i < RC_NJ; i++) {
            uint8_t id = SERVO_ID[i];
            uint8_t err = 0;
            if (!sbus_ping(id, &err)) {
                printf("[BUS] ID=%d  *** NO ANSWER ***\n", id);
                continue;
            }
            uint16_t model = 0, pos = 0;
            uint8_t volt = 0;
            uint8_t temp = 0;
            bool ok_model = sbus_read_u16(id, SB_REG_MODEL, &model);
            bool ok_pos = sbus_read_u16(id, SB_REG_PRESENT_POSITION, &pos);
            (void)sbus_read_u8(id, SB_REG_PRESENT_VOLTAGE, &volt);
            (void)sbus_read_u8(id, SB_REG_PRESENT_TEMPERATURE, &temp);
            if (!ok_model || model != SB_STS3215_MODEL) {
                printf("[BUS] ID=%d  Model=%u (expect %d)  *** WRONG MODEL ***\n", id, model,
                       SB_STS3215_MODEL);
                continue;
            }
            if(!ok_pos||pos>4095||temp>=100) {
                printf("[BUS] ID=%u invalid initial feedback; startup refused\n",id);continue;
            }
            start_pos[i] = pos;
            uint16_t runtime_torque_limit=0;
            bool torque_limit_ok=sbus_read_u16(id,SB_REG_TORQUE_LIMIT,&runtime_torque_limit);
            uint8_t pid_bytes[8]={0};bool pid_ok=sbus_read_reg(id,21,8,pid_bytes);
            printf("[BUSCFG] ID=%u runtime_torque_limit=%s%u reg21..28=%s",
                id,torque_limit_ok?"":"unreadable:",runtime_torque_limit,pid_ok?"":"unreadable:");
            for(unsigned k=0;k<8;k++)printf("%02X",pid_bytes[k]);
            printf("\n");
            uint8_t torque_existing=0;uint16_t previous_goal=0,confirm_goal=0;
            bool torque_read=sbus_read_u8(id,SB_REG_TORQUE_ENABLE,&torque_existing);
            torque_released[i]=torque_read&&torque_existing==0;
            hot_valid[i]=torque_read&&torque_existing==1&&
                sbus_read_u16(id,SB_REG_GOAL_POSITION,&previous_goal)&&
                sbus_read_u16(id,SB_REG_GOAL_POSITION,&confirm_goal)&&previous_goal==confirm_goal&&
                previous_goal<=4095&&abs((int)previous_goal-(int)pos)<180;
            hot_goal[i]=previous_goal;
            /* ★ 把 EEPROM 里的**真实机械/软件限位**读出来对比。
             * 这一条是专门为了回答"某个轴是不是已经撞到硬限位"：
             *   位置 = 86 而 Min_Angle 远小于 86 → 不是限位问题，要往别处查；
             *   位置 ≈ Min_Angle            → 确实顶在限位上了，
             *                                 那么**绝不能朝那个方向再给指令**。 */
            uint16_t amin = 0, amax = 0, tmax = 0;
            bool ok_min = sbus_read_u16(id, SB_REG_MIN_ANGLE, &amin);
            bool ok_max = sbus_read_u16(id, SB_REG_MAX_ANGLE, &amax);
            /* Startup only: retry a rejected read, never substitute guessed
             * EEPROM boundaries or write them back to the servo. */
            for(int retry=0;retry<2&&(!ok_min||!ok_max);retry++) {
                if(!ok_min)ok_min=sbus_read_u16(id,SB_REG_MIN_ANGLE,&amin);
                if(!ok_max)ok_max=sbus_read_u16(id,SB_REG_MAX_ANGLE,&amax);
            }
            (void)sbus_read_u16(id, SB_REG_MAX_TORQUE, &tmax);
            /* 出厂没设过限位时是 0..4095（全量程）—— 那等于"没有保护"，
             * 必须记成无效，否则软限位会以为自己很安全。 */
            bool this_lim_ok = ok_min && ok_max && !(amin == 0 && amax == 4095) && (amin < amax);
            lim_min[i] = amin;
            lim_max[i] = amax;
            lim_ok[i] = this_lim_ok;
            printf("[BUS] ID=%d  OK  Model=%u  POS=%4u  %s  %uC\n", id, model, pos,
                   (volt >= 30 && volt <= 90) ? "VOLT_OK" : "VOLT_INVALID", temp);
            if (ok_min && ok_max) {
                int near_lo = (int)pos - (int)amin;
                int near_hi = (int)amax - (int)pos;
                printf("[BUS]      limit_min=%4u limit_max=%4u torque_max=%3u |"
                       " headroom: lo=%+5d hi=%+5d %s\n",
                       amin, amax, tmax, near_lo, near_hi,
                       (near_lo < 40 || near_hi < 40) ? "*** AT/BEYOND LIMIT ***" : "");
            } else {
                printf("[BUS]      (limit registers unreadable)\n");
            }
            found++;
            esp_rom_delay_us(SB_INTERFRAME_US);
        }
        if (found == RC_NJ) {
            break;
        }
        /* ★ 打印原始字节数：这是"物理层通不通"的唯一直接证据。
         *   0 字节  → 线/UART 配置问题（ESP32 的 TX 或 RX 没接上）
         *   有字节但 NO ANSWER → 通了，是波特率/协议/收发方向问题 */
        printf("[BUS] raw bytes seen this pass = %lu, timeouts = %lu\n",
               (unsigned long)sbus_raw_bytes_take(), (unsigned long)sbus_txn_timeouts());
        sbus_raw_hex_dump();
        printf("[BUS] only %d/%d answered -- retrying in 1s\n", found, RC_NJ);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (found != RC_NJ) {
        printf("\n[BUS] *** %d/%d servos only. TORQUE WILL NOT BE ENABLED. ***\n", found, RC_NJ);
        /* ★ 2026-10-02 实测更正：这一条**不是**原因，别再看它了。
         * 当时现象是"ESP32 扫 0/6、PC 从舵机板 USB 扫 6/6"，我据此判断是
         * 板载 CH343 压住总线，把它写成了第 1 条。**随后实测推翻了它**：
         *   USB 拔掉 → 仍 0/6；USB 插回 → 仍 0/6。
         * 所以 USB 插不插与这个故障无关（SOP §7 记的"插不插都行"是对的）。
         * 真正的判据是下面这条 **[BUS] raw bytes**：0 字节 = 物理层没通。 */
        printf("[BUS]  1) look at 'raw bytes seen' above: 0 => the ESP32 receives NOTHING,\n");
        printf("[BUS]     so it is the WIRING/UART path, not baud/protocol. Check in order:\n");
        printf("[BUS]     a) board TTL header TX -> GPIO18, RX -> [983R] -> GPIO17, GND<->GND\n");
        printf("[BUS]       (NOT the A/B/V2/G header -- that is RS485 and gives 0 bytes)\n");
        printf("[BUS]     b) the 983R resistor is IN SERIES on the TX path and is a common\n");
        printf("[BUS]       break point -- buzz it end to end\n");
        printf("[BUS]     c) swap GPIO17 / GPIO18 and retry (both orientations have worked)\n");
        printf("[BUS]     d) try a whole different set of jumper wires (a wire can be broken\n");
        printf("[BUS]       inside while looking perfect)\n");
        printf("[BUS]  2) if raw bytes > 0 but no answer: baud/protocol -- try 500000 / 250000\n");
        printf("[BUS]  3) is the 5V 4A supply ON?  do the servos twitch at power-up?\n");
        return false;
    }

    /* ★ 增量式：指令位置对齐到**实测位置**（rc_sync_to_meas）。
     * 这样开力矩那一刻 "目标 = 实际位置"，臂纹丝不动。
     * ❗ 增量式下不再需要：锚点、软限位、chan_half、行程自测。
     *    那四样东西都是"绝对映射"时代为了猜行程而加的，而今天一半的 bug
     *    （腕部撞死、底座单向）都出在猜出来的那个数上。
     *    现在"撞限位"与"卡住"被统一成同一件事，由堵转保护处理。 */
    for (int i = 0; i < RC_NJ; i++) {
        s_j[i].meas = (int16_t)start_pos[i];
        s_j[i].meas_ok = true;
        s_j[i].cmd_pos = (float)start_pos[i];
        s_j[i].vel = 0.0f;
        s_j[i].rate = 0.0f;
        s_j[i].carry = 0.0f;
        s_j[i].ch_init = false; /* 第一帧只记基准，不产生增量 */
    }
    rc_sync_to_meas(s_j);
    for(int i=0;i<RC_NJ;i++)if(hot_valid[i]) {
        // The servo retained torque and this exact goal across an ESP-only
        // restart. Preserve its existing load-supporting command error.
        s_j[i].cmd_pos=(float)hot_goal[i];
        printf("[BUS] hot hold CH%d cmd=%u meas=%u retained\n",i+1,hot_goal[i],start_pos[i]);
    }
    /* Level constraints use real EEPROM boundaries plus a local reference window. */
    s_level_limits_ok = true;
    for (int i = 0; i < 3; ++i) {
        int ch = i + 1;
        s_level_lo[i] = (float)lim_min[ch] + 40.0f;
        s_level_hi[i] = (float)lim_max[ch] - 40.0f;
        if (!lim_ok[ch] || s_level_lo[i] >= s_level_hi[i]) s_level_limits_ok = false;
    }

    /* ★ CH6 夹爪：把 joint6 的 EEPROM 限位交给控制律，转成**档位制绝对位置**
     * （遥控端 Z 手势给出 6 档，通道值 q∈[-1,+1] 在 [合, 开] 间线性插值成绝对目标；
     *  开 = limit_max，合 = limit_min − RC_GRIP_PRESS 越过量 = 夹持力）。
     * 修的是增量式三个结构性 bug：钳位吃位移、"合"只回到上电起点、误差归零夹不紧。
     * 限位读不出来就退回增量式（能用，但行程打折），并明确警告。 */
    if (lim_ok[RC_GRIP_CH]) {
        rc_set_grip_range((float)lim_max[RC_GRIP_CH], (float)lim_min[RC_GRIP_CH]);
        printf("[GRIP] stepped absolute: open=%u close=%u (press %.0f tick beyond)\n",
               lim_max[RC_GRIP_CH], lim_min[RC_GRIP_CH], (double)RC_GRIP_PRESS);
    } else {
        printf("[GRIP] *** WARNING: joint6 EEPROM limits unreadable -- CH6 falls back to\n");
        printf("[GRIP]           incremental mode (partial stroke, weak grip) ***\n");
    }
    printf("[LIM] incremental mode: no soft limits, no travel table.\n");
    printf("[LIM]   the arm stops at a mechanical stop by itself (position stops rising),\n");
    printf("[LIM]   and the stall guard releases that joint if it keeps not moving.\n");
    printf("[LIM]   start pose (tick):");
    for (int i = 0; i < RC_NJ; i++) {
        printf(" %u", start_pos[i]);
    }
    printf("\n");
    s_boot_torque_released=true;
    for(int i=0;i<RC_NJ;i++)if(!torque_released[i])s_boot_torque_released=false;
    s_bus_ok = true;
    s_cal_ok = true;
    return true;
}

/* ============================== 回读任务 ==============================
 * 单独一个任务读位置，**不放进 200 Hz 控制循环**：
 * 一次读失败最长要等 25 ms（SB_READ_TIMEOUT_MS），放进控制循环就会
 * 让同步写跟着卡 25 ms → 关节一顿一顿。分开之后：
 *   控制循环只做"算 + 发同步写"（0.5 ms 级），读位置慢一点无所谓。
 * 两者共用总线，靠 sbus_lock() 互斥。 */
static void read_task(void *arg)
{
    (void)arg;
    while (true) {
        for (int i = 0; i < RC_NJ; i++) {
            uint16_t pos = 0, spd = 0, load = 0;
            uint8_t v = 0, t = 0;
            sbus_lock();
            /* ★ 一条 READ(56,8) 拿到位置+速度+负载+电压+温度，且用**短超时**：
             * 短超时是守住 200 Hz 的关键（长超时会把控制循环拖到 46 Hz）。 */
            bool ok = sbus_read_status(SERVO_ID[i], &pos, &spd, &load, &v, &t,
                                       SB_READ_TIMEOUT_SHORT_MS);
            sbus_unlock();
            if (ok) {
                s_j[i].meas = (int16_t)pos;
                s_j[i].meas_ok = true;
                s_joint_temp[i] = t;
                s_joint_read_us[i] = esp_timer_get_time();
                s_read_ok++;
                if (t > s_temp_max) {
                    s_temp_max = (int16_t)t;
                }
                /* ⚠️ 电压寄存器实测**会返回离谱值**（开机扫描时读到 643.6 V，
                 *    运行中在 13.8 V 与 53.8 V 之间跳），而同一时刻的位置/温度
                 *    都是好的 —— 说明是这一个字节的偶发破坏，不是逻辑错。
                 *    STS3215 在 5–7.4 V 供电下，合理范围 3.0–9.0 V 之外一律
                 *    判为无效（填 0 = 未知），**绝不把假数字当电压上报**。 */
                if (v >= 30 && v <= 90) {
                    s_vbus_mv = (uint16_t)(v * 100);
                }
            } else {
                s_j[i].meas_ok = false;
                s_read_fail++;
            }
            /* ★ 舵机电源掉了要能一眼看出来。
             * 2026-10-02 实测：只断开舵机那路 5V 时，**USB 供的电还够舵机逻辑电路应答**，
             * 所以串口"看起来一切正常"，但读操作会大面积超时 → 控制循环被拖垮
             * （实测 200 Hz → 46 Hz）、现场表现是"臂顶住不动、推了没反应"。
             * 这个坑我花了一轮才定位，所以固化成一个显式告警。
             * 判据：最近 200 次回读里失败超过一半 —— 那就不可能是偶发丢帧了。 */
            s_rd_hist[s_rd_hist_i] = ok ? 1u : 0u;
            s_rd_hist_i = (s_rd_hist_i + 1) % RD_HIST_N;
            if (s_rd_hist_i == 0) {
                int good = 0;
                for (int k = 0; k < RD_HIST_N; k++) {
                    good += (int)s_rd_hist[k];
                }
                bool rail_down = (good * 2 < RD_HIST_N);
                s_rail_ok = !rail_down; /* ★ 供堵转保护判断"读数可不可信" */
                if (rail_down && !s_rail_warned) {
                    s_rail_warned = true;
                    printf("\n[PWR] *** most servo reads are failing (%d/%d ok) ***\n",
                           good, RD_HIST_N);
                    printf("[PWR] servo rail (5V 4A) is probably OFF or sagging while USB\n");
                    printf("[PWR] keeps the servo LOGIC alive -- the bus still answers, so this\n");
                    printf("[PWR] looks like a software fault but is not. Check the 5V supply.\n\n");
                } else if (!rail_down && s_rail_warned) {
                    s_rail_warned = false;
                    printf("[PWR] servo reads recovered (%d/%d ok)\n", good, RD_HIST_N);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(READ_PERIOD_MS)>0?pdMS_TO_TICKS(READ_PERIOD_MS):1);
        }
    }
}

/* ============================== 遥测帧发送 ============================== */
static void send_telemetry(void)
{
    rt_joint_t j;
    memset(&j, 0, sizeof(j));
    j.seq = ++s_tx_joint;
    j.t_send_ms = (uint32_t)(esp_timer_get_time() / 1000);
    /* ★ 把控制帧的 t_send_ms 原样回抄 —— 遥控端用它算 RTT。
     * 这样全程**没有跨板时钟减法**（两块板 esp_timer 纪元不同）。 */
    rt_ctrl_t snap;
    memcpy(&snap, (const void *)&s_ctrl_shadow, sizeof(snap));
    j.echo_t_ms = snap.t_send_ms;
    j.mode = s_mode;
    j.status_flags = 0;
    if (s_torque_on) {
        j.status_flags |= RT_ST_TORQUE_ON;
    }
    if (s_ctrl_last_us != 0 &&
        (esp_timer_get_time() - s_ctrl_last_us) < (int64_t)LINK_TIMEOUT_MS * 1000) {
        j.status_flags |= RT_ST_LINK_OK;
    }
    if (s_bus_ok) {
        j.status_flags |= RT_ST_BUS_OK;
    }
    if (s_cal_ok) {
        j.status_flags |= RT_ST_CAL_OK;
    }
    if (s_estop_latched) {
        j.status_flags |= RT_ST_ESTOP;
    }
    if (s_clamped_g) {
        j.status_flags |= RT_ST_CLAMPED;
    }
    j.rx_seq_last = (uint16_t)(s_rx_last_seq < 0 ? 0 : s_rx_last_seq);
    j.rx_ok = s_rx_ok;
    j.rx_lost = s_rx_lost;
    j.rx_age_ms = (s_ctrl_last_us == 0)
                      ? 0xFFFFFFFFu
                      : (uint32_t)((esp_timer_get_time() - s_ctrl_last_us) / 1000);
    j.vbus_mv = s_vbus_mv;
    j.temp_max_c = s_temp_max;
    /* valid_mask：位 i = 该轴最近一次回读成功。
     * ★ 遥控端必须看这一位：回读失败时 pos[i] 是**上一次的旧值**，
     *   直接显示会让人以为"臂卡住了"，其实是总线丢了一帧。 */
    j.valid_mask = 0;
    for (int i = 0; i < RC_NJ; i++) {
        if (s_j[i].meas_ok) {
            j.valid_mask |= (uint16_t)(1u << i);
        }
        j.pos[i] = s_j[i].meas;
        j.tgt[i] = (int16_t)(s_j[i].cmd_pos + 0.5f);
    }
    j.teach_frames = 0; /* 示教缓冲还没做（P3 创新项）；先如实报 0 */
    j.teach_state = 0;
    uint8_t buf[RT_PKT_JOINT_LEN];
    rt_joint_pack(buf, &j);
    const uint8_t *dst = s_peer_ready ? s_peer_mac : NULL;
    if (!dst) {
        return; /* 还没学到遥控端 MAC：不发（省得往广播里灌无用的包） */
    }
    esp_now_send(dst, buf, sizeof(buf));
}

/* ============================== 主流程 ============================== */
/* 链路年龄的显示口径：遥控链路没帧时 age_us 是 INT64_MAX 哨兵值，直接打会
 * 输出 9223372036854775 ms。FPGA 视觉链路在跑时改用它的帧年龄，仍没有就
 * 打 "--" —— 只是显示，不参与 link_ok 判定。 */
static const char *link_age_text(int64_t age_us, char *buf, size_t n)
{
#if FPGA_LINK_ENABLE
    if (age_us >= INT64_MAX / 2) {
        fpga_link_stats_t fs;
        fpga_link_get_stats(&fs);
        if (fs.last_frame_age_us >= 0) age_us = fs.last_frame_age_us;
    }
#endif
    if (age_us >= INT64_MAX / 2) snprintf(buf, n, "--");
    else snprintf(buf, n, "%lld", (long long)(age_us / 1000));
    return buf;
}

void app_main(void)
{
    rt_level_init(&s_level); /* Restore historical reference after healthy feedback. */
    rt_height_init(&s_height);
    /* 高度天花板：85−10 = 75 mm（2026-11-02 按现场要求恢复）。
     * 这是"别让工具飘离物体高度"的必要约束——IBVS 只控径向与角度，
     * 高度完全靠这条压着；放开到 290 时工具飘到 119 mm，闭爪就是夹空气。
     * 之前它在 60~64 mm 处 402 次整步拒绝，根因是"指令预测高度"被跟随差虚高，
     * 已在 rt_height.c 改为只用实测姿态判据，不再是这条上限本身的问题。 */
    (void)rt_height_set_limit(&s_height, 85.0f, 10.0f);
    (void)rt_level_set_source(&s_level, true); /* Backlash invalidated constant command bias. */
    (void)rt_level_set_speed(&s_level, 400.0f, 800.0f);
    (void)rt_level_set_filter(&s_level, 20.0f);
    /* 腕关节没有给固定值，仍用“参考±窗口”；800 太小会把 CH3 卡在 ~2042，
     * 放到 1200 才够 CH3 走到 1800 一侧，仍由 EEPROM 余量兜底。 */
    (void)rt_level_set_range(&s_level, 600.0f, 1200.0f);
    (void)rt_level_set_tolerance(&s_level, 10.0f); /* 5° → 10°：腕到边界时的建模余量，给水平耦合多留空间 */
    /* 现场要求：肩/肘用固定工作区间（CH2 1400..3200，CH3 1500..3200），
     * 不再跟着开机位置走。EEPROM 余量（±40 tick）仍然是最后一道，
     * 所以 CH2 实际到 3153、CH3 到 3045，表里写 3200 只是“允许到”。
     * CH2：2600 -> 3000（补采探针被顶死）-> 3200（REACH 时 cmd 停在 2999）。 */
    (void)rt_level_set_fixed(&s_level, true,
        (const float[]){1400.0f, 1500.0f}, (const float[]){3200.0f, 3200.0f});
    print_banner();
#if FPGA_LINK_ENABLE
    /* FPGA 视觉桥：只安装 UART2 + 控制台行读取，默认不使能 */
    (void)fpga_link_start();
#endif
    vTaskDelay(pdMS_TO_TICKS(BOOT_DELAY_MS));
    if (!proto_selftest()) {
        printf("[RX] *** protocol self-test FAILED -- refusing to enable torque. ***\n");
        printf("[RX] this almost always means the two firmwares have different rt_protocol.h\n");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }

    /* --- 1) 只读阶段：扫总线、读初始位置（此时力矩是关的，臂不会动）--- */
    sbus_init();
    if (!bus_bringup()) {
        printf("[RX] refusing to enable torque. Fix the bus, then reset the board.\n");
        /* 不重启、不退出：留在循环里，方便反复看串口诊断 */
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            printf("[RX] bus retry...\n");
            if (bus_bringup()) {
                break;
            }
        }
    }

    /* --- 2) 无线（先起无线再开力矩：万一无线有问题，臂还是"没上力"的状态）--- */
    wifi_espnow_init();

    /* --- 3) 开力矩：此刻目标 = 当前位置，所以臂**不会动**，只是"变硬" --- */
    printf("\n[TORQUE] restoring detected hold/release state\n");
    sbus_lock();
    if(!s_boot_torque_released)sbus_torque_all(SERVO_ID, RC_NJ, true);
    sbus_unlock();
    s_torque_on = !s_boot_torque_released;
    if(s_boot_torque_released) {
        s_service_hold=true;s_boot_profile_pending=false;
        for(int i=0;i<RC_NJ;i++)s_service_pose[i]=s_j[i].cmd_pos;
        rc_grip_auto_hold_current(s_j);
        s_auto.reason="torque released; KEY2 holds, KEY3 restores and starts";
        printf("[TORQUE] OFF retained; KEY2 holds current pose; KEY3 restores then starts\n");
    } else printf("[TORQUE] ON; existing hold retained\n");

    /* --- 3b) 关节能动性自检：小幅移动再回原位，回答"哪个轴被机械挡住" --- */

    /* --- 4) 回读任务（绑核 1；主控制循环在 core 0，互不抢）--- */
    xTaskCreatePinnedToCore(read_task, "read_servo", READ_TASK_STACK, NULL, 5, NULL, 1);

    /* --- 5) 200 Hz 控制循环 --- */
    int64_t t_prev = esp_timer_get_time();
    int64_t t_draw = t_prev, t_tel = t_prev, t_log = t_prev;
    int rate_total = 0, rate_ok = 0;
    rt_ctrl_t last_ctrl;
    memset(&last_ctrl, 0, sizeof(last_ctrl));
    bool have_ctrl = false;
    bool s_link_was_ok = false;
    bool s_estop_logged = false;
    bool s_anchor_captured = false;
    bool s_clamped = false;
    int64_t t_clamp_log = t_prev;
    uint32_t s_clamp_sig = 0xFFFFFFFFu; /* 上次报过的"贴在限位上的轴"掩码；0xFFFFFFFF = 还没报过 */
    uint32_t height_feedback_flags = 0;
    int64_t t_height_warning = -1000000, t_level_warning = -1000000;
    bool carry_was_requested = false;

    printf("\n[RX] entering 200 Hz control loop. Waiting for control frames...\n");
    if (s_peer_ready) {
        char m[20];
        mac_str(s_peer_mac, m);
        printf("[RX] remote = %s\n", m);
    }

    while (true) {
        int64_t t_loop = esp_timer_get_time();

        /* --- 取一帧控制（快照）--- */
        if((s_boot_profile_pending||s_service_hold)&&!fpga_pos_selected()) {
            fpga_pos_auto_gates(true,false,false);
            fpga_pos_set_gain_pct(0,0,0);
            fpga_pos_set_active(true);
        }
        fpga_pos_service();
        bool radio_fresh=s_ctrl_fresh;
        bool fresh = radio_fresh;
        if (fresh) {
            memcpy(&last_ctrl, (const void *)&s_ctrl_shadow, sizeof(last_ctrl));
            s_ctrl_fresh = false;
            have_ctrl = true;
        }
        rt_ctrl_t radio_snapshot;
        memcpy(&radio_snapshot,(const void *)&s_ctrl_shadow,sizeof(radio_snapshot));
        int64_t age_us = (s_ctrl_last_us == 0) ? INT64_MAX : (t_loop - s_ctrl_last_us);
        bool link_ok = (age_us < (int64_t)LINK_TIMEOUT_MS * 1000);

#if FPGA_LINK_ENABLE
        /* FPGA 视觉桥：只覆盖 last_ctrl，不新增控制路径。
         * fpga_link_get() 内部已做：使能判断 / 100ms stale / 软限位 / 越界丢帧。 */
        rt_ctrl_t fc;
        if (fpga_link_get(&fc)) {
            /* ★ 2026-10-28 现场抓到的坑：**FPGA 帧会把夹爪夹一下**。
             * 原因：桥里 memset 后 ch[5] 恒为 0，而夹爪是"档位制绝对位置"，
             * 0 = **中位**（不是"别动"）。一旦 s_grip_engaged 为真，
             * 每个 FPGA 帧都命令夹爪去中位 —— 表现为"没发指令它自己在夹"。
             * 这里把 CH6 还原成遥控端最后一帧的值：**夹爪始终只归遥控管**。 */
            fc.ch[5] = s_ctrl_shadow.ch[5];
            /* Preserve the real gripper engagement button too, so the first
             * deliberate close request works during test/FPGA arm control. */
            fc.buttons |= s_ctrl_shadow.buttons & RT_BTN_Z;
            memcpy(&last_ctrl, &fc, sizeof(last_ctrl));
            have_ctrl = true;
            link_ok = true;
            fresh = true;
        }
#endif

#if FPGA_LINK_ENABLE
        /* ★ 2026-10-28：位置式把 **FPGA 帧**当链路源。
         * 否则遥控端一断（ESP-NOW 丢）就走 HOLD 分支，视觉伺服跟着失效 ——
         * 现场踩过：遥控器连不上，臂就完全不跟了。 */
        if (fpga_pos_selected() && fpga_link_frame_fresh()) {
            have_ctrl = true;
            link_ok = true;
            fresh = true;
        }

        /* ★ 位置式 arm：以当前实测姿态为基准（目标=实测，上电/使能不跳变） */
        {
            int16_t mp_now[RC_NJ];
            for (int i = 0; i < RC_NJ; i++) {
                mp_now[i] = s_j[i].meas;
            }
            fpga_pos_set_meas(mp_now);   /* 抗饱和：目标只许领先实测一个 LAG */
            /* 水平约束的可用区间也递进去：肩顶到边界时径向需求转给肘。 */
            if (s_level.enabled)
                fpga_pos_set_limits(s_level.lo[0], s_level.hi[0], s_level.lo[1], s_level.hi[1]);
            else
                fpga_pos_set_limits(FPGA_POS_MIN, FPGA_POS_MAX, FPGA_POS_MIN, FPGA_POS_MAX);
        }
        if (fpga_pos_needs_arm()) {
            int16_t mp[RC_NJ];
            bool all_ok = true;
            for (int i = 0; i < RC_NJ; i++) {
                mp[i] = s_j[i].meas;
                if (!s_j[i].meas_ok) {
                    all_ok = false;
                }
            }
            if (all_ok) {
                float command_pose[RC_NJ];
                for(int i=0;i<RC_NJ;i++)command_pose[i]=s_j[i].cmd_pos;
                fpga_pos_arm_at_command(mp,command_pose);
                fpga_frame_t initial_frame;
                if(fpga_link_get_last(&initial_frame))s_visual_seq=initial_frame.seq;
                s_visual_closed=rc_grip_closed_requested();
                fpga_pos_note_armed();
            }
        }
#endif

        if(s_position_was_selected && !fpga_pos_selected()) {
            rc_zero_rate(s_j);
            vision_pick_cancel(&s_pick);
        }
        s_position_was_selected=fpga_pos_selected();

        /* --- 状态机 --- */
        if (s_estop_latched) {
            /* ★ 急停锁存：**冻结目标 + 力矩照常使能**。
             * 为什么不是松力矩：按下红色按钮时人要的是"立刻停住别动"，
             * 而急停往往发生在"已经看出要撞了"的瞬间 —— 那时松力矩
             * 就是让带重力负载的臂自由落体，正好是急停本该防止的事故。
             * 松力矩是另一个独立动作（演示结束后手动做）。 */
            /* ★ 速率通道必须归零：急停时人可能还推着摇杆，
             * 不归零 rc_step 会继续把 rate 积分进 cmd_pos → 解冻瞬间猛冲。 */
            rc_zero_rate(s_j);
            if (!s_estop_logged) {
                s_estop_logged = true;
                printf("[SAFE] *** ESTOP LATCHED: frozen, torque still ON (arm holds) ***\n");
            }
        } else if (!s_torque_on) {
            /* 没有力矩（只可能是启动失败或显式松力矩）：什么都别发。
             * 速率同样归零，防止 cmd_pos 在无力矩期间被暗中积分跑飞。 */
            rc_zero_rate(s_j);
        } else if (have_ctrl && link_ok) {
            if (fresh) {
                float pp[RC_NJ];
                if (fpga_pos_selected()) {
                    /* 位置式：桥给绝对目标。未 arm 完成时**什么都不做**
                     * （宁可这一拍不动，也不能回落到速率式去积分）。 */
                    /* Real radio keeps ownership of CH5/CH6 and command events.
                     * Only selected visual joints are overridden below. */
                    if(radio_fresh) rc_apply_ctrl(&radio_snapshot,s_j);
                    if(age_us>=(int64_t)LINK_TIMEOUT_MS*1000)
                        for(int i=0;i<RC_NJ;i++)s_j[i].rate=s_j[i].vel=0;
                    if (fpga_pos_get_float(pp)) {
                        for (int i=0;i<3;i++) if(fpga_pos_axis_owned((unsigned)i)) {
                            s_j[i].cmd_pos=(float)pp[i];s_j[i].rate=s_j[i].vel=0;
                        }
                    }
                } else {
                /* ★ 增量式（2026-10-02 晚定案）：不再有锚点、软限位、行程表。
                 * 第一帧只记通道基准、不产生增量 → **上电位移恒为 0**，
                 * 无论遥控端此刻通道值是多少。见 rt_control.h 顶部说明。 */
                if (!s_anchor_captured) {
                    s_anchor_captured = true;
                    printf("[ARM] incremental mode engaged. cmd_pos (tick):");
                    for (int i = 0; i < RC_NJ; i++) {
                        printf(" %d", (int)(s_j[i].cmd_pos + 0.5f));
                    }
                    printf("\n[ARM] first-frame channels: %d %d %d %d %d %d (no delta on frame 1)\n",
                           last_ctrl.ch[0], last_ctrl.ch[1], last_ctrl.ch[2], last_ctrl.ch[3],
                           last_ctrl.ch[4], last_ctrl.ch[5]);
                }
                    rc_apply_ctrl(&last_ctrl, s_j);
                }
            }
        } else {
            /* ★ 失联：冻结目标 + 保持力矩 = 顶住当前姿态，不下滑。
             * 恢复后**不会**自动跟动 —— 走 rc_ramp_arm() 那条突跳保护路径，
             * 否则遥控端在这段时间里推过的值会一下子全部兑现（臂会猛冲）。 */
            /* ★ 失联立刻把速率归零（每个失联周期都调，开销可忽略）：
             * 推住的摇杆不能变成"失联后臂继续跑 300ms"。
             * 增量通道无此问题 —— 没有新帧就没有增量，天然冻结。 */
            rc_zero_rate(s_j);
            if (s_mode != RT_MODE_FAILSAFE) {
                s_mode = RT_MODE_FAILSAFE;
                char age_txt[24];
                printf("[SAFE] link lost (age=%s ms) -> HOLD position, torque stays ON\n",
                       link_age_text(age_us, age_txt, sizeof age_txt));
                (void)0; /* 增量式：不需要冲坡保护（增量本身就是平滑的） */
            }
        }

        /* --- 命令事件：只在 cmd_seq 变化时执行一次 --- */
        if (radio_fresh && radio_snapshot.cmd_code != RT_CMD_NONE) {
            bool fire = false;
            if (!s_cmd_seq_init) {
                s_cmd_seq_init = true;
                s_last_cmd_seq = radio_snapshot.cmd_seq;
            } else if (radio_snapshot.cmd_seq != s_last_cmd_seq) {
                s_last_cmd_seq = radio_snapshot.cmd_seq;
                fire = true;
            }
            if (fire) {
                switch (radio_snapshot.cmd_code) {
                case RT_CMD_ESTOP:
                    s_estop_latched = true;
                    s_estop_logged = false;
                    (void)0; /* 增量式：不需要冲坡保护（增量本身就是平滑的） */
                    printf("[SAFE] *** ESTOP command (seq=%u) -> freeze + hold torque ***\n",
                           radio_snapshot.cmd_seq);
                    break;
                case RT_CMD_CLEAR_ESTOP:
                    s_estop_latched = false;
                    (void)0; /* 增量式：不需要冲坡保护（增量本身就是平滑的） */ /* 恢复时也走斜坡，别猛冲 */
                    printf("[SAFE] estop cleared (seq=%u) -> resuming, ramped\n",
                           radio_snapshot.cmd_seq);
                    break;
                case RT_CMD_YAW_RECENTER:
                    printf("[CMD] yaw recenter ignored (done on the remote side)\n");
                    break;
                default:
                    printf("[CMD] code=%u seq=%u arg=%u -- not implemented yet\n",
                           radio_snapshot.cmd_code, radio_snapshot.cmd_seq, radio_snapshot.cmd_arg);
                    break;
                }
            }
        }

        /* --- 链路恢复检测：从 LOST 回到 OK 时挂斜坡 --- */
        if (link_ok && !s_link_was_ok) {
            (void)0; /* 增量式：不需要冲坡保护（增量本身就是平滑的） */
            char age_txt[24];
            printf("[LINK] back OK (age=%s ms) -> ramped resume\n",
                   link_age_text(age_us, age_txt, sizeof age_txt));
        }
        s_link_was_ok = link_ok;

        /* --- 控制律：把 pos 朝 tgt 平滑推进 --- */
        int64_t now = esp_timer_get_time();
        float dt_ms = (float)(now - t_prev) / 1000.0f;
        t_prev = now;
        /* LEVEL_BEGIN: command-space horizontal compensation, main-loop owned. */
#if FPGA_LINK_ENABLE
        if(s_boot_profile_pending) {
            if(s_key_restart) {
                s_boot_prepare_ms+=(unsigned)fminf(dt_ms,50);
                if(s_boot_prepare_ms>5000) {
                    s_boot_profile_pending=false;s_key_restart=false;s_service_hold=true;
                    for(int i=0;i<RC_NJ;i++)s_service_pose[i]=s_j[i].cmd_pos;
                    s_auto.reason="startup refused: feedback or constraints not ready within 5s";
                    printf("[KEY3] %s\n",s_auto.reason);
                }
            }
            bool ready=s_level_limits_ok&&s_bus_ok&&s_torque_on&&s_rail_ok&&
                !s_estop_latched&&have_ctrl&&link_ok&&rc_grip_active();
            for(int i=0;i<RC_NJ;i++)if(!s_j[i].meas_ok||!s_joint_read_us[i]||
                now-s_joint_read_us[i]>250000||s_joint_temp[i]>=65||s_stall_latched[i]||
                fabsf(s_j[i].rate)>1||fabsf(s_j[i].cmd_pos-s_j[i].meas)>=160)ready=false;
            if(s_key_restart&&dt_ms>50)ready=false;
            if(s_boot_profile_pending&&s_key_restart&&ready) {
                fpga_frame_t startup_frame;
                if(!fpga_link_get_last(&startup_frame)||!fpga_link_frame_fresh()||!startup_frame.visual_enabled) {
                    s_boot_profile_pending=false;s_key_restart=false;s_service_hold=true;
                    s_auto.reason="startup stopped: KEY3 disabled or vision link stale";
                    ready=false;
                } else {
                    float proposed[3];
                    int recovery=boot_profile_margin_step(
                        (float[]){s_j[1].cmd_pos,s_j[2].cmd_pos,s_j[3].cmd_pos},
                        (float[]){s_j[1].meas,s_j[2].meas,s_j[3].meas},
                        s_level_lo,s_level_hi,fminf(dt_ms,5)/5,proposed);
                    if(recovery<0) {
                        s_boot_profile_pending=false;s_key_restart=false;s_service_hold=true;
                        s_auto.reason="startup refused: outside EEPROM bounds or excessive following error";
                        ready=false;printf("[KEY3] %s\n",s_auto.reason);
                    } else if(recovery>0) {
                        ready=false;s_service_hold=true;
                        for(int i=0;i<3;i++)s_j[i+1].cmd_pos=s_service_pose[i+1]=proposed[i];
                        s_auto.reason="startup recovering inward from EEPROM safety margin";
                        if(!s_margin_recover_logged) {
                            printf("[KEY3] recovering inward: q=%d,%d,%d allowed=%.0f..%.0f,%.0f..%.0f,%.0f..%.0f\n",
                                s_j[1].meas,s_j[2].meas,s_j[3].meas,(double)s_level_lo[0],(double)s_level_hi[0],
                                (double)s_level_lo[1],(double)s_level_hi[1],(double)s_level_lo[2],(double)s_level_hi[2]);
                            s_margin_recover_logged=true;
                        }
                    }
                }
            }
            s_boot_healthy_ms=ready?s_boot_healthy_ms+(unsigned)fminf(dt_ms,50):0;
            if(s_boot_profile_pending&&s_boot_healthy_ms>=500) {
                s_boot_profile_pending=false;
                bool ok=!rc_grip_closed_requested()&&
                    boot_profile_restore(&s_level,&s_height,
                        (float[]){s_j[1].cmd_pos,s_j[2].cmd_pos,s_j[3].cmd_pos},
                        (float[]){s_j[1].meas,s_j[2].meas},s_level_lo,s_level_hi)&&
                    rc_grip_auto_set_contact(1734,50);
                if(ok)ok=vision_auto_arm(&s_auto,BOOT_LIFT_DEGREES);
                if(ok&&s_key_restart)s_auto.key3_was_off=true;
                if(ok)s_service_hold=false;
                s_key_restart=false;
                if(!ok){
                    rt_level_disable(&s_level);rt_height_disable(&s_height);
                    s_service_hold=true;
                    for(int i=0;i<RC_NJ;i++)s_service_pose[i]=s_j[i].cmd_pos;
                    s_auto.reason="startup refused: current pose cannot restore historical constraints";
                }
                printf("[BOOT] historical level/table %s; lift=45deg COMMAND_ONLY; KEY3 OFF->ON required\n",ok?"restored":"REFUSED");
            }
        }
        rt_level_request_t lr;
        if (fpga_link_take_level_request(&lr)) {
            if (lr.action == LEVEL_REQ_OFF || lr.action == LEVEL_REQ_ON ||
                lr.action == LEVEL_REQ_TRIM || lr.action == LEVEL_REQ_SIGN ||
                lr.action == LEVEL_REQ_SOURCE || lr.action == LEVEL_REQ_TOLERANCE) {
                rt_height_disable(&s_height);
                s_height.referenced = false;
            }
            if (lr.action == LEVEL_REQ_OFF) {
                rt_level_disable(&s_level);
                s_j[3].rate = s_j[3].vel = 0.0f;
                printf("[LEVEL] OFF; wrist released, command position retained\n");
            } else if (lr.action == LEVEL_REQ_ON) {
                bool ready = s_level_limits_ok && s_bus_ok && s_torque_on &&
                             s_rail_ok && !s_estop_latched;
                float q[3];
                for (int i = 0; i < 3; ++i) {
                    q[i] = s_j[i + 1].cmd_pos;
                    if (!s_j[i + 1].meas_ok || s_stall_latched[i + 1] ||
                        s_joint_read_us[i + 1] == 0 ||
                        now - s_joint_read_us[i + 1] > 250000 || s_joint_temp[i + 1] >= 65 ||
                        fabsf(s_j[i + 1].rate) > 0.1f ||
                        fabsf(q[i] - (float)s_j[i + 1].meas) >= RC_STALL_TICKS) ready = false;
                }
                if (ready && rt_level_enable(&s_level, q, s_level_lo, s_level_hi)) {
                    float measured[2] = {(float)s_j[1].meas, (float)s_j[2].meas};
                    if (!rt_level_seed_measurement(&s_level, measured)) rt_level_disable(&s_level);
                    printf("[LEVEL] ON: current pose is horizontal reference, cmd %.1f %.1f %.1f\n",
                           (double)q[0], (double)q[1], (double)q[2]);
                    printf("[LEVEL] captured cmd-meas bias %.1f %.1f %.1f tick; signs %.0f %.0f\n",
                           (double)(q[0] - s_j[1].meas), (double)(q[1] - s_j[2].meas),
                           (double)(q[2] - s_j[3].meas), (double)s_level.sign[0], (double)s_level.sign[1]);
                } else printf("[LEVEL] ON refused: stop inputs, check bus/limits, or OFF before recapture\n");
            } else if (lr.action == LEVEL_REQ_TRIM) {
                printf("[LEVEL] trim %s (absolute wrist trim %.2f deg)\n",
                       rt_level_set_trim(&s_level, lr.a) ? "accepted" : "refused", (double)lr.a);
            } else if (lr.action == LEVEL_REQ_SIGN) {
                printf("[LEVEL] sign %s (change only while OFF)\n",
                       rt_level_set_sign(&s_level, lr.a, lr.b) ? "accepted" : "refused");
            } else if (lr.action == LEVEL_REQ_SOURCE) {
                printf("[LEVEL] source %s (change only while OFF)\n",
                       rt_level_set_source(&s_level, lr.a != 0.0f) ? "accepted" : "refused");
            } else if (lr.action == LEVEL_REQ_SPEED) {
                printf("[LEVEL] speed %s\n", rt_level_set_speed(&s_level, lr.a, lr.b) ? "accepted" : "refused");
            } else if (lr.action == LEVEL_REQ_FILTER) {
                printf("[LEVEL] filter %s\n", rt_level_set_filter(&s_level, lr.a) ? "accepted" : "refused");
            } else if (lr.action == LEVEL_REQ_RANGE) {
                printf("[LEVEL] range %s\n", rt_level_set_range(&s_level, lr.a, lr.b) ? "accepted" : "refused");
            } else if (lr.action == LEVEL_REQ_TOLERANCE) {
                printf("[LEVEL] tolerance %s\n", rt_level_set_tolerance(&s_level, lr.a) ? "accepted" : "refused");
            }
            printf("[LEVEL] enabled=%d limited=%d sign=%.0f,%.0f trim=%.2f source=%s\n",
                   s_level.enabled, s_level.limited, (double)s_level.sign[0],
                   (double)s_level.sign[1], (double)s_level.trim_deg,
                   s_level.measured_source ? "meas-filtered" : "cmd");
            printf("[LEVEL] speed arm=%.0f wrist=%.0f tick/s filter=%.0fms window=+/-%.0f,+/-%.0f tick\n",
                   (double)s_level.arm_tps, (double)s_level.wrist_tps, (double)s_level.filter_ms,
                   (double)s_level.arm_window, (double)s_level.wrist_window);
            printf("[LEVEL] tolerance=+/-%.1f deg relaxed=%d slack=%+.2f deg (modeled)\n",
                   (double)s_level.tolerance_deg, s_level.relaxed, (double)s_level.slack_deg);
            printf("[GRASP] closed_command=%d carry_height=%d carry_level_angle=%d ch6=%d "
                   "absolute=%d preserve_hold_error=%d\n",
                   rc_grip_closed_requested(), s_height.carry_mode, s_level.carry_mode,
                   (int)last_ctrl.ch[RC_GRIP_CH], rc_grip_active(), rc_grip_preserve_hold_error());
            if (s_level.enabled && s_level.measured_source && s_level.measured_ready) {
                float ideal = s_level.ref[2]
                    - s_level.sign[0] * (s_level.measured_filtered[0] - s_level.measured_ref[0])
                    - s_level.sign[1] * (s_level.measured_filtered[1] - s_level.measured_ref[1])
                    + s_level.trim_deg * LEVEL_TICKS_PER_DEG;
                float bounded = fminf(s_level.hi[2], fmaxf(s_level.lo[2], ideal));
                printf("[LEVEL] measured ideal_wrist=%.2f slack_now=%+.2f deg (modeled)\n",
                       (double)ideal, (double)((bounded - ideal) / LEVEL_TICKS_PER_DEG));
            }
            for (int i = 1; i <= 3; ++i)
                printf("[LEVEL] CH%d cmd=%.2f meas=%d bias=%.2f temp=%u age=%lldms\n",
                       i + 1, (double)s_j[i].cmd_pos, s_j[i].meas,
                       (double)(s_j[i].cmd_pos - s_j[i].meas), (unsigned)s_joint_temp[i],
                       s_joint_read_us[i] == 0 ? -1LL : (long long)((now - s_joint_read_us[i]) / 1000));
            if (s_level.enabled)
                for (int i = 0; i < 3; ++i)
                    printf("[LEVEL] CH%d effective range %.0f..%.0f tick (%s)\n",
                           i + 2, (double)s_level.lo[i], (double)s_level.hi[i],
                           (s_level.fixed && i < 2) ? "fixed table, EEPROM margin retained"
                                                    : "EEPROM margin retained");
            if (s_level.fixed)
                printf("[LEVEL] fixed table CH2 %.0f..%.0f CH3 %.0f..%.0f\n",
                       (double)s_level.fixed_lo[0], (double)s_level.fixed_hi[0],
                       (double)s_level.fixed_lo[1], (double)s_level.fixed_hi[1]);
        }

        rt_height_request_t hr;
        if (fpga_link_take_height_request(&hr)) {
            float measured[2] = {(float)s_j[1].meas, (float)s_j[2].meas};
            bool stationary = true;
            for (int i = 1; i <= 3; ++i)
                if (fabsf(s_j[i].rate) > 0.1f || fabsf(s_j[i].vel) > 0.1f) stationary = false;
            bool feedback_ok = s_bus_ok && s_torque_on && s_rail_ok && !s_estop_latched;
            for (int i = 1; i <= 3; ++i)
                if (!s_j[i].meas_ok || s_stall_latched[i] ||
                    s_joint_read_us[i] == 0 || now - s_joint_read_us[i] > 250000 ||
                    s_joint_temp[i] >= 65) feedback_ok = false;
            bool accepted = false;
            switch (hr.action) {
            case HEIGHT_REQ_OFF:
                rt_height_disable(&s_height);
                accepted = true;
                break;
            case HEIGHT_REQ_ON:
                accepted = s_level.enabled && stationary && feedback_ok &&
                           rt_height_enable(&s_height,
                               (float[]){s_j[1].cmd_pos, s_j[2].cmd_pos}, measured);
                break;
            case HEIGHT_REQ_MODEL:
                accepted = !s_height.enabled && stationary &&
                           rt_height_set_model(&s_height, hr.a, hr.b, hr.c);
                break;
            case HEIGHT_REQ_REF:
                accepted = !s_height.enabled && s_level.enabled && stationary && feedback_ok &&
                           rt_height_reference(&s_height, measured, hr.a);
                break;
            case HEIGHT_REQ_LIMIT:
                accepted = rt_height_set_limit(&s_height, hr.a, hr.b);
                break;
            case HEIGHT_REQ_STAT:
                accepted = true;
                break;
            }
            printf("[HEIGHT] %s model=%d ref=%d enabled=%d limited=%d fault=%d "
                   "measured=%.1f predicted=%.1f limit=%.1f margin=%.1fmm carry=%d\n",
                   accepted ? "accepted" : "refused", s_height.model_ready,
                   s_height.referenced, s_height.enabled, s_height.limited, s_height.fault,
                   (double)s_height.measured_mm, (double)s_height.predicted_mm,
                   (double)s_height.limit_mm, (double)s_height.margin_mm, s_height.carry_mode);
            printf("[HEIGHT] feedback flags=0x%08lx (ctrl/link/rail/estop/torque/level: bits0..5; "
                   "CH2..4 read:8..10 stale:12..14 hot:16..18)\n",
                   (unsigned long)height_feedback_flags);
        }
#endif
        if (s_level.enabled && (s_estop_latched || !s_torque_on ||
            s_stall_latched[1] || s_stall_latched[2] || s_stall_latched[3])) {
            rt_level_disable(&s_level);
            rc_zero_rate(s_j);
            printf("[LEVEL] OFF due to stop/torque/stall; re-enable explicitly\n");
        }
        bool auto_snapshot=false,console_start=false;
        int probe_axis=-1;float probe_delta=0;
        vision_request_t vr;
        if(fpga_link_take_vision_request(&vr)) {
            if(vr.action==1) {
                bool added=fpga_pos_add_sample(vr.a,vr.b,vr.c,vr.d,vr.e);
                printf("[B] sample %s; count=%u\n",added?"accepted":"refused",fpga_pos_sample_count());
            }
            else if(vr.action==2){fpga_pos_clear_samples();printf("[B] samples cleared\n");}
            else if(vr.action==3)printf("[PICK] arm %s: %.1f deg; waits for next deliberate close\n",
                vision_pick_arm(&s_pick,vr.a)?"accepted":"refused",(double)vr.a);
            else if(vr.action==4){vision_pick_cancel(&s_pick);printf("[PICK] cancelled; hold retained\n");}
            else if(vr.action==5||vr.action==10) {
                bool ready=fpga_pos_selected()&&s_level.enabled&&s_height.enabled&&s_height.referenced&&
                    rc_grip_active()&&!rc_grip_closed_requested()&&!vision_auto_running(&s_auto)&&!s_probe.active;
                bool ok=ready&&(vr.action==10?vision_auto_finish(&s_auto,vr.a):vision_auto_arm(&s_auto,vr.a));
                if(ok) {
                    vision_pick_cancel(&s_pick);
                    /* Explicit console arming may continue the operator's
                     * existing physical KEY3 enable; boot still requires a new edge. */
                    fpga_frame_t armed_frame;
                    if(fpga_link_get_last(&armed_frame)&&fpga_link_frame_fresh()&&armed_frame.visual_enabled)
                        s_auto.key3_was_off=true;
                }
                printf("[AUTO] arm %s; lift=%.1f; waits for KEY3; no contact sensor\n",ok?"accepted":"refused",(double)vr.a);
                if(ok&&vr.action==10)printf("[AUTO] operator-confirmed endpoint; freeze approach, settle before close\n");
            }
            else if(vr.action==6) {
                s_calibrate.active=false;s_boot_profile_pending=false;
                s_probe.active=false;
                vision_auto_cancel(&s_auto,"operator stopped");memset(&s_auto,0,sizeof(s_auto));
                vision_pick_cancel(&s_pick);rc_grip_auto_freeze(s_j);
                if(!rc_grip_closed_requested())rc_grip_auto_release();
                fpga_pos_set_active(false);rc_zero_rate(s_j);
                printf("[AUTO] stopped; targets and torque retained\n");
            }
            else if(vr.action==9) {
                bool ok=!vision_auto_running(&s_auto)&&!s_auto.close_issued&&!rc_grip_closed_requested();
                if(ok){s_probe.active=false;fpga_pos_set_gain_pct(0,0,0);fpga_pos_set_active(true);}
                printf("[AUTO] prepare %s; all visual gains zero\n",ok?"accepted":"refused");
            }
            else if(vr.action==12) {
                fpga_frame_t held_frame={0};
                bool ok=fpga_pos_selected()&&s_level.enabled&&s_height.enabled&&s_height.referenced&&
                    !vision_auto_running(&s_auto)&&!s_probe.active&&have_ctrl&&link_ok&&s_rail_ok&&
                    !s_estop_latched&&s_torque_on&&rc_grip_auto_hold_ready(s_j)&&
                    fpga_link_get_last(&held_frame)&&fpga_link_frame_fresh()&&held_frame.v2&&held_frame.visual_enabled;
                for(int i=0;i<RC_NJ;i++)if(!s_j[i].meas_ok||!s_joint_read_us[i]||
                    now-s_joint_read_us[i]>250000||s_joint_temp[i]>=65||s_stall_latched[i]||fabsf(s_j[i].rate)>1)ok=false;
                if(ok)ok=vision_auto_resume_lift(&s_auto,vr.a,&held_frame);
                if(ok)ok=vision_pick_resume(&s_pick,vr.a)&&rc_grip_auto_command(s_j,true);
                if(!ok&&s_auto.state==VA_LIFT)vision_auto_cancel(&s_auto,"resume refused");
                printf("[AUTO] resume lift %s: %.1f deg; existing grip target retained\n",ok?"accepted":"refused",(double)vr.a);
            }
            else if(vr.action==11) {
                bool ok=!vision_auto_running(&s_auto)&&!rc_grip_closed_requested()&&!s_probe.active&&
                    s_j[RC_GRIP_CH].meas_ok&&s_joint_read_us[RC_GRIP_CH]>0&&
                    now-s_joint_read_us[RC_GRIP_CH]<250000&&s_j[RC_GRIP_CH].meas>vr.a&&
                    rc_grip_auto_set_contact(vr.a,vr.b);
                printf("[AUTO] contact target %s: observed=%.1f preload=%.1f close=%.1f; RAM only\n",
                    ok?"accepted":"refused",(double)vr.a,(double)vr.b,(double)(vr.a-vr.b));
            }
            else if(vr.action==7)auto_snapshot=true;
            else if(vr.action==14)console_start=true;
            else if(vr.action==8){probe_axis=(int)vr.a-1;probe_delta=vr.b;}

        }
        bool carry_requested = rc_grip_closed_requested();
        bool visual_selected=fpga_pos_selected();
        bool visual_feedback=have_ctrl && link_ok && s_rail_ok && !s_estop_latched && s_torque_on;
        for(int i=0;i<RC_NJ;i++) if(!s_j[i].meas_ok || s_joint_read_us[i]==0 ||
            now-s_joint_read_us[i]>250000 || s_joint_temp[i]>=65 || s_stall_latched[i]) visual_feedback=false;
        bool neutral_radio=true;
        if(radio_fresh)for(int i=0;i<5;i++)if(abs(radio_snapshot.ch[i])>6000)neutral_radio=false;
        bool manual_override=radio_fresh&&(!neutral_radio||(radio_snapshot.buttons&RT_BTN_Z));
        fpga_frame_t vf={0};
        bool have_vf=fpga_link_get_last(&vf) && fpga_link_frame_fresh();
        bool new_vf=have_vf&&vf.seq!=s_visual_seq;
        if(have_vf&&(vf.seq!=s_operator_seq||console_start)) {
            unsigned keys=0;
            if(vf.seq!=s_operator_seq) {
                s_operator_seq=vf.seq;
                keys=operator_keys_step(&s_operator_keys,vf.key_events_valid,
                    vf.key_events,vf.visual_enabled,now);
            }
            bool restartable=s_service_hold||(s_auto.state==VA_ABORT&&!rc_grip_closed_requested());
            if(console_start) {
                bool accepted=vf.key_events_valid&&vf.visual_enabled&&restartable&&
                    !vision_auto_running(&s_auto)&&!s_probe.active&&!rc_grip_closed_requested();
                if(accepted)keys|=OP_KEY_START;
                printf("[AUTO] start %s; physical KEY3 must remain enabled\n",accepted?"accepted":"refused");
            }
            operator_action_t action=operator_keys_action(keys,s_torque_on,restartable);
            bool start_from_released=action==OP_START_RESTORE;
            if((keys&(OP_KEY_OPEN|OP_KEY_TORQUE))||start_from_released) {
                s_calibrate.active=false;s_probe.active=false;s_boot_profile_pending=false;
                s_adaptive_grip=false;s_service_open=false;s_key_restart=false;
                vision_pick_cancel(&s_pick);memset(&s_auto,0,sizeof(s_auto));
                fpga_pos_auto_gates(true,false,false);rc_zero_rate(s_j);
                if(!s_service_hold)for(int i=0;i<RC_NJ;i++)s_service_pose[i]=s_j[i].cmd_pos;
                s_service_hold=true;
                if((keys&OP_KEY_TORQUE)||start_from_released) {
                    bool turn_on=start_from_released||!s_torque_on,ok=true;
                    s_torque_error[0]=0;
                    s_level.enabled=false;s_height.enabled=false;
                    rc_grip_auto_hold_current(s_j);
                    uint16_t hold[RC_NJ];
                    sbus_lock();
                    if(turn_on) {
                        if(!s_bus_ok||!s_rail_ok||s_estop_latched){ok=false;torque_failure(0,"bus/rail/estop not ready");}
                        for(int i=0;i<RC_NJ;i++) {
                            uint16_t position=0;
                            if(s_joint_temp[i]>=65){ok=false;torque_failure(i+1,"temperature limit");}
                            if(s_stall_latched[i]){ok=false;torque_failure(i+1,"stall latched");}
                            if(!sbus_read_u16(SERVO_ID[i],SB_REG_PRESENT_POSITION,&position)||position>4095)
                                {ok=false;torque_failure(i+1,"position feedback rejected");}
                            hold[i]=position;
                        }
                        if(ok&&!sbus_sync_write_goal(SERVO_ID,RC_NJ,hold,GOAL_ACC,GOAL_SPEED))
                            {ok=false;torque_failure(0,"hold target write failed");}
                        if(ok)for(int i=0;i<RC_NJ;i++) {
                            uint16_t written=0;
                            if(!sbus_read_u16(SERVO_ID[i],SB_REG_GOAL_POSITION,&written)||written!=hold[i])
                                {ok=false;torque_failure(i+1,"hold target readback mismatch");}
                        }
                        if(ok) {
                            for(int i=0;i<RC_NJ;i++) {
                                s_j[i].meas=(int16_t)hold[i];s_j[i].meas_ok=true;
                                s_j[i].cmd_pos=s_service_pose[i]=hold[i];
                                s_j[i].rate=s_j[i].vel=s_j[i].carry=0;
                                s_joint_read_us[i]=esp_timer_get_time();
                            }
                            sbus_torque_all(SERVO_ID,RC_NJ,true);
                            for(int i=0;i<RC_NJ;i++) {
                                uint8_t enabled=0;
                                if(!sbus_read_u8(SERVO_ID[i],SB_REG_TORQUE_ENABLE,&enabled)||enabled!=1)
                                    {ok=false;torque_failure(i+1,"enable verification failed");}
                            }
                        }
                    }
                    if(!turn_on||!ok) {
                        sbus_torque_all(SERVO_ID,RC_NJ,false);
                        for(int i=0;i<RC_NJ;i++) {
                            uint8_t enabled=1;
                            if(!sbus_read_u8(SERVO_ID[i],SB_REG_TORQUE_ENABLE,&enabled)||enabled!=0)
                                {ok=false;torque_failure(i+1,"release verification failed");}
                        }
                    }
                    s_torque_on=turn_on&&ok;
                    sbus_unlock();
                    fpga_pos_clear_samples();s_visual_closed=false;
                    if(s_torque_on) {
                        int16_t current[RC_NJ];float accepted[RC_NJ];
                        for(int i=0;i<RC_NJ;i++){current[i]=s_j[i].meas;accepted[i]=s_j[i].cmd_pos;}
                        fpga_pos_arm_at_command(current,accepted);
                    }
                    s_auto.reason=s_torque_on?"holding current position; press KEY3 to start":
                        (ok?"torque released; KEY2 holds, KEY3 restores and starts":s_torque_error);
                    if(!ok)printf("[TORQUE] %s\n",s_torque_error);
                    printf("[KEY2] torque=%s result=%s; current position hold; limits unchanged\n",
                        s_torque_on?"ON":"OFF",ok?"accepted":"refused");
                    if(start_from_released&&s_torque_on) {
                        s_service_hold=true;s_boot_profile_pending=true;s_boot_healthy_ms=0;
                        s_boot_prepare_ms=0;s_key_restart=true;s_margin_recover_logged=false;
                        s_auto.reason="KEY3 restored torque; preparing historical constraints";
                        printf("[KEY3] current pose captured; torque verified; preparing startup\n");
                    }
                } else {
                    s_service_open=visual_feedback&&s_torque_on&&rc_grip_auto_open_to(s_j,1978);
                    printf("[KEY1] open %s; arm goals held; automatic cycle cancelled\n",s_service_open?"accepted":"refused");
                }
            } else if(action==OP_START_READY) {
                if(s_torque_on&&!s_estop_latched) {
                    s_calibrate.active=false;s_probe.active=false;vision_pick_cancel(&s_pick);
                    memset(&s_auto,0,sizeof(s_auto));
                    if(!s_service_hold)for(int i=0;i<RC_NJ;i++)s_service_pose[i]=s_j[i].cmd_pos;
                    s_service_open=false;s_service_hold=true;s_adaptive_grip=false;
                    rt_level_disable(&s_level);rt_height_disable(&s_height);
                    rc_grip_auto_hold_current(s_j);rc_zero_rate(s_j);
                    s_boot_profile_pending=true;s_boot_healthy_ms=0;s_boot_prepare_ms=0;s_key_restart=true;s_margin_recover_logged=false;
                    s_auto.reason="KEY3 preparing historical constraints";
                    printf("[KEY3] restart requested; restore historical constraints before motion\n");
                } else printf("[KEY3] refused: restore torque with KEY2 first\n");
            }
        }
        if(!visual_selected || (radio_fresh&&!neutral_radio))vision_pick_cancel(&s_pick);
        if(manual_override&&s_auto.state!=VA_IDLE) {
            s_calibrate.active=false;
            vision_auto_cancel(&s_auto,"manual override");vision_pick_cancel(&s_pick);
            fpga_pos_set_active(false);rc_grip_auto_release();rc_zero_rate(s_j);
        }
        if(visual_selected && carry_requested) s_visual_closed=true;
        if(visual_selected && !carry_requested && have_vf && !vf.visual_enabled) s_visual_closed=false;
        if(have_vf)fpga_pos_observe_frame(&vf);
        fpga_pos_set_height_model(s_height.enabled&&s_level.enabled,
            s_height.model.off2,s_height.model.off3,s_height.model.sign);
        if(new_vf&&visual_feedback&&!carry_requested&&!manual_override)fpga_pos_track_frame(&vf);
        if(new_vf) {
            s_auto_motion=0;
            for(int i=0;i<4;i++) {
                s_auto_motion=fmaxf(s_auto_motion,fabsf((float)s_j[i].meas-s_auto_motion_ref[i]));
                s_auto_motion_ref[i]=s_j[i].meas;
            }
        }
        float speed=0,error=0;
        for(int i=0;i<4;i++) {
            speed=fmaxf(speed,fabsf(s_j[i].rate));
            error=fmaxf(error,fabsf(s_j[i].cmd_pos-s_j[i].meas));
        }
        if(s_auto.state==VA_WAIT&&s_auto.key3_was_off&&have_vf&&vf.visual_enabled&&
           visual_feedback&&!manual_override) {
            if(!rc_grip_auto_open_to(s_j,1978))vision_auto_cancel(&s_auto,"startup open refused");
        }
        if(s_auto.state==VA_WAIT&&(!visual_feedback||!have_vf||!vf.visual_enabled))rc_grip_auto_freeze(s_j);
        vision_auto_state_t auto_before=s_auto.state;
        vision_auto_output_t ao=vision_auto_step(&s_auto,&(vision_auto_input_t){
            .configured=rc_grip_active(),.selected=visual_selected,
            .constraints_ready=s_level.enabled&&s_height.enabled&&s_height.referenced,
            .healthy=visual_feedback,.manual_override=manual_override,
            .frame_fresh=have_vf,.new_frame=new_vf,.map_ready=fpga_pos_map_ready(),
            .jaws_open=!carry_requested&&s_j[5].meas>=1962&&fabsf(s_j[5].cmd_pos-1978)<=4&&fabsf(s_j[5].rate)<1,
            .calibrating=s_calibrate.active,
            .gripper_settled=carry_requested&&(!s_adaptive_grip||s_grip_seek.state==VG_READY)&&fabsf(s_j[RC_GRIP_CH].rate)<1,
            .lift_done=s_pick.state==VP_DONE,.lift_aborted=s_pick.state==VP_ABORT,
            .joint_speed=speed,.measured_motion=s_auto_motion,.position_error=error,
            .expected_offset=0,.dt_ms=(unsigned)dt_ms,.frame=have_vf?&vf:NULL});
        bool automatic=s_auto.state!=VA_IDLE;
        if(s_calibrate.active) {
            ao.pan=ao.reach=ao.close=ao.arm_lift=false;ao.freeze=true;
        }
        if(ao.calibrate&&!s_calibrate.active) {
            vision_calibrate_start(&s_calibrate);
            printf("[B-AUTO] begin bounded measured local calibration\n");
        }
        fpga_pos_auto_gates(automatic,ao.pan,ao.reach);
        if(ao.arm_lift&&!vision_pick_arm(&s_pick,s_auto.lift_degrees))vision_auto_cancel(&s_auto,"lift arm refused");
        if(ao.close&&s_auto.state!=VA_ABORT) {
            s_adaptive_grip=true;s_grip_seek_read_us=s_joint_read_us[RC_GRIP_CH];
            if(!vision_grip_begin(&s_grip_seek,s_j[RC_GRIP_CH].cmd_pos,
                s_j[RC_GRIP_CH].meas,rc_grip_floor()))vision_auto_cancel(&s_auto,"gripper seek refused");
            s_visual_closed=true;
        }
        if(s_auto.close_issued&&s_auto.state!=VA_ABORT&&s_auto.state!=VA_IDLE) {
            if(s_adaptive_grip) {
                bool fresh=s_grip_seek_read_us!=s_joint_read_us[RC_GRIP_CH];
                vision_grip_step(&s_grip_seek,visual_feedback,fresh,(unsigned)dt_ms,
                    s_j[RC_GRIP_CH].cmd_pos,s_j[RC_GRIP_CH].meas);
                if(fresh)s_grip_seek_read_us=s_joint_read_us[RC_GRIP_CH];
                if(s_grip_seek.state==VG_FAILED) {
                    printf("[GRIP] seek failed: elapsed=%ums accepted=%.0f measured=%d contact=%.0f target=%.0f late=%u\n",
                           s_grip_seek.elapsed_ms,(double)s_j[RC_GRIP_CH].cmd_pos,s_j[RC_GRIP_CH].meas,
                           (double)s_grip_seek.contact,(double)s_grip_seek.target,s_grip_seek.late_cycles);
                    vision_auto_cancel(&s_auto,"gripper resistance not established");
                }
                else if(visual_feedback) {
                    if(!rc_grip_auto_close_to(s_j,s_grip_seek.target,100))
                        vision_auto_cancel(&s_auto,"gripper seek command refused");
                } else rc_grip_auto_freeze(s_j);
            } else if(visual_feedback)(void)rc_grip_auto_command(s_j,true);
        }
        if(s_auto.state==VA_ABORT){vision_pick_cancel(&s_pick);rc_grip_auto_freeze(s_j);rc_zero_rate(s_j);}
        if(s_service_hold) {
            rc_zero_rate(s_j);
            if(s_service_open) {
                if(!visual_feedback||!s_torque_on||!rc_grip_auto_open_to(s_j,1978)) {
                    s_service_open=false;rc_grip_auto_freeze(s_j);
                    printf("[KEY1] open stopped: feedback or command rejected\n");
                } else if(fabsf(s_j[5].cmd_pos-1978)<=3&&fabsf((float)s_j[5].meas-1978)<=12) {
                    s_service_open=false;rc_grip_auto_freeze(s_j);
                    printf("[KEY1] open complete\n");
                }
            }
        }
        if(auto_before!=s_auto.state)printf("[AUTO] state=%s reason=%s\n",vision_auto_name(s_auto.state),s_auto.reason?s_auto.reason:"none");
        carry_requested=rc_grip_closed_requested();
        float lift_proposal=s_j[1].cmd_pos;
        vision_pick_state_t previous_pick=s_pick.state;
        bool lift_owned=vision_pick_step(&s_pick,carry_requested&&(!s_adaptive_grip||s_grip_seek.state==VG_READY)&&fabsf(s_j[RC_GRIP_CH].rate)<1.0f,
            visual_selected&&have_vf&&vf.v2&&vf.visual_enabled&&s_level.enabled&&s_height.enabled&&neutral_radio,
            visual_feedback&&neutral_radio,s_j[1].cmd_pos,(float)s_j[1].meas,(float)s_j[2].meas,
            s_height.model.off2,s_height.model.off3,s_height.model.sign,(unsigned)dt_ms,&lift_proposal);
        if(previous_pick!=s_pick.state)printf("[PICK] state=%d CH2 start=%.1f target=%.1f\n",
            (int)s_pick.state,(double)s_pick.start,(double)s_pick.target);
        /* 把高度上限提前喂给径向求解器：超过上限只许下降；离上限还远才允许"边伸边抬"；
         * 中间带维持等高切线。不这样做，求解器在上限处会被逐帧整步拒绝而卡死。 */
        {
            float hceil = s_height.limit_mm - s_height.margin_mm;
            int hmode = 0;
            if(s_height.enabled && isfinite(s_height.measured_mm)) {
                if(s_height.measured_mm > hceil) hmode = 2;
                else if(s_height.measured_mm < hceil - 10.0f) hmode = 1;
            }
            vision_reach_set_height_mode(hmode);
        }
        if(visual_selected) {
            if(new_vf) {
                s_visual_seq=vf.seq;
                if(visual_feedback&&!s_visual_closed&&!carry_requested&&!manual_override)fpga_pos_step_frame(&vf);
            }
            float visual_target[RC_NJ];
            if(fpga_pos_get_float(visual_target))for(int i=0;i<3;i++)if(fpga_pos_axis_owned((unsigned)i)) {
                s_j[i].cmd_pos=visual_target[i];s_j[i].rate=s_j[i].vel=0;
            }
        }
        if(probe_axis>=0) {
            bool ok=(probe_axis==1||probe_axis==2)&&fabsf(probe_delta)<=20&&visual_selected&&
                s_level.enabled&&s_height.enabled&&visual_feedback&&!carry_requested&&
                !vision_auto_running(&s_auto)&&!s_auto.close_issued&&have_vf&&vf.found_red;
            float next=ok?s_j[probe_axis].cmd_pos+probe_delta:0;
            ok=ok&&next>=s_level.lo[probe_axis-1]&&next<=s_level.hi[probe_axis-1]&&
                fabsf(next-s_j[probe_axis].meas)<100;
            if(ok)ok=vision_probe_arm(&s_probe,(unsigned)probe_axis,s_j[probe_axis].cmd_pos,probe_delta);
            printf("[PROBE] %s CH%d delta=%.2f; common constraints still apply\n",ok?"accepted":"refused",probe_axis+1,(double)probe_delta);
        }
        if(s_probe.active) {
            float proposal;
            bool permitted=visual_selected&&s_level.enabled&&s_height.enabled&&!carry_requested&&
                !manual_override&&!vision_auto_running(&s_auto)&&!s_auto.close_issued;
            unsigned axis=s_probe.axis;
            if(vision_probe_step(&s_probe,dt_ms,permitted,visual_feedback&&have_vf&&vf.found_red,
                                 s_j[axis].cmd_pos,&proposal)) {
                s_j[axis].cmd_pos=proposal;s_j[axis].rate=s_j[axis].vel=0;
            }
        }
        if(s_calibrate.active) {
            float proposal[2];
            bool permitted=s_auto.state==VA_REACH&&visual_selected&&s_level.enabled&&s_height.enabled&&
                !carry_requested&&!manual_override&&(!have_vf||vf.visual_enabled);
            bool used=vision_calibrate_step(&s_calibrate,have_vf?&vf:NULL,new_vf,
                visual_feedback&&have_vf,permitted,(unsigned)dt_ms,
                (float[]){s_j[0].meas,s_j[1].meas,s_j[2].meas},
                (float[]){s_j[1].cmd_pos,s_j[2].cmd_pos},proposal);
            if(used)for(int i=0;i<2;i++){s_j[i+1].cmd_pos=proposal[i];s_j[i+1].rate=s_j[i+1].vel=0;}
            if(s_calibrate.done) {
                bool added=fpga_pos_add_sample(s_j[0].meas,s_j[1].meas,s_j[2].meas,
                    s_calibrate.gradient[0],s_calibrate.gradient[1]);
                printf("[B-AUTO] sample %s: q=%d,%d,%d gradient=%.5f,%.5f\n",added?"accepted":"refused",
                    s_j[0].meas,s_j[1].meas,s_j[2].meas,(double)s_calibrate.gradient[0],(double)s_calibrate.gradient[1]);
                if(!added)vision_auto_cancel(&s_auto,"automatic B sample refused");
                s_calibrate.done=false;
            }
            if(s_calibrate.failed) {
                if(s_calibrate.phase<3)snprintf(s_calibration_error,sizeof(s_calibration_error),
                    "B calibration phase=%u line=%u corrections=%u cmd=%.1f,%.1f q=%d,%d origin=%.1f,%.1f",
                    s_calibrate.phase,s_calibrate.failure_line,s_calibrate.corrections,
                    (double)s_j[1].cmd_pos,(double)s_j[2].cmd_pos,s_j[1].meas,s_j[2].meas,
                    (double)s_calibrate.origin[1],(double)s_calibrate.origin[2]);
                else snprintf(s_calibration_error,sizeof(s_calibration_error),"B calibration phase=%u line=%u move=%.1f,%.1f,%.1f,%.1f radial=%.1f,%.1f",
                    s_calibrate.phase,s_calibrate.failure_line,(double)s_calibrate.measured_matrix[0],
                    (double)s_calibrate.measured_matrix[1],(double)s_calibrate.measured_matrix[2],
                    (double)s_calibrate.measured_matrix[3],(double)s_calibrate.radius_delta[0],(double)s_calibrate.radius_delta[1]);
                if(s_auto.state!=VA_ABORT)vision_auto_cancel(&s_auto,s_calibration_error);
            }
        }
        if(lift_owned){s_j[1].cmd_pos=lift_proposal;s_j[1].rate=s_j[1].vel=0;}
        if(auto_snapshot) {
            unsigned stall_mask=0;
            for(int i=0;i<RC_NJ;i++)if(s_stall_latched[i])stall_mask|=1u<<i;
            char auto_text[1536];
            snprintf(auto_text,sizeof(auto_text),"[AUTOJSON] {\"state\":\"%s\",\"healthy\":%d,\"selected\":%d,\"level\":%d,\"height_on\":%d,\"height\":%.3f,"
                "\"closed\":%d,\"absolute_grip\":%d,\"frame_fresh\":%d,\"enabled\":%d,\"found\":%d,\"samples\":%u,\"online_samples\":%u,\"map_ready\":%d,"
                "\"pole\":[%d,%d],\"red\":[%u,%u],\"blue\":[%u,%u],\"goal\":[%u,%u],\"seq\":%lu,"
                "\"q\":[%d,%d,%d,%d,%d,%d],\"cmd\":[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],"
                "\"temp\":[%u,%u,%u,%u,%u,%u],\"read_age\":[%lld,%lld,%lld,%lld,%lld,%lld],\"torque\":%d,\"service_hold\":%d,\"service_open\":%d,\"key_protocol\":%d,\"grip_state\":%d,\"stall_mask\":%u,\"reason\":\"%s\"}\n",
                vision_auto_name(s_auto.state),(int)visual_feedback,(int)visual_selected,(int)s_level.enabled,(int)s_height.enabled,
                s_height.enabled?(double)s_height.measured_mm:0.0,(int)carry_requested,(int)rc_grip_active(),(int)have_vf,
                (int)vf.visual_enabled,(int)(vf.found_red&&vf.found_blue),fpga_pos_sample_count(),fpga_pos_online_count(),(int)fpga_pos_map_ready(),
                vf.pole_x,vf.pole_y,vf.red_x,vf.red_y,vf.blue_x,vf.blue_y,vf.goal_x,vf.goal_y,(unsigned long)vf.seq,
                s_j[0].meas,s_j[1].meas,s_j[2].meas,s_j[3].meas,s_j[4].meas,s_j[5].meas,
                (double)s_j[0].cmd_pos,(double)s_j[1].cmd_pos,(double)s_j[2].cmd_pos,(double)s_j[3].cmd_pos,(double)s_j[4].cmd_pos,(double)s_j[5].cmd_pos,
                s_joint_temp[0],s_joint_temp[1],s_joint_temp[2],s_joint_temp[3],s_joint_temp[4],s_joint_temp[5],
                (long long)((now-s_joint_read_us[0])/1000),(long long)((now-s_joint_read_us[1])/1000),
                (long long)((now-s_joint_read_us[2])/1000),(long long)((now-s_joint_read_us[3])/1000),
                (long long)((now-s_joint_read_us[4])/1000),(long long)((now-s_joint_read_us[5])/1000),
                (int)s_torque_on,(int)s_service_hold,(int)s_service_open,(int)vf.key_events_valid,(int)s_grip_seek.state,stall_mask,
                s_auto.reason?s_auto.reason:"none");
            fpga_link_auto_report(auto_text);
        }

        if (carry_requested != carry_was_requested) {
            carry_was_requested = carry_requested;
            printf("[GRASP] %s; joint limits and feedback protection retained\n",
                   carry_requested ? "CARRY: step 0, height/pitch gates bypassed"
                                   : "APPROACH: nonzero step, height/pitch gates restored");
        }
        s_level.carry_mode = s_level.enabled && carry_requested;
        s_height.carry_mode = s_height.enabled && carry_requested;
        if (s_level.enabled) s_j[3].rate = s_j[3].vel = 0.0f;
        rc_step(s_j, dt_ms); /* Shoulder/elbow commands are generated first. */
        if (s_height.enabled && !s_level.enabled) {
            rt_height_disable(&s_height);
            s_height.referenced = false;
            printf("[HEIGHT] OFF because level is OFF; re-reference explicitly\n");
        }
        if (s_height.enabled) {
            float q[2] = {s_j[1].cmd_pos, s_j[2].cmd_pos};
            float measured[2] = {(float)s_j[1].meas, (float)s_j[2].meas};
            height_feedback_flags = (!have_ctrl ? 1u : 0u) | (!link_ok ? 2u : 0u) |
                (!s_rail_ok ? 4u : 0u) | (s_estop_latched ? 8u : 0u) |
                (!s_torque_on ? 16u : 0u) | (!s_level.enabled ? 32u : 0u);
            for (int i = 1; i <= 3; ++i) {
                if (!s_j[i].meas_ok) height_feedback_flags |= 1u << (7 + i);
                if (s_joint_read_us[i] == 0 || now - s_joint_read_us[i] > 250000)
                    height_feedback_flags |= 1u << (11 + i);
                if (s_joint_temp[i] >= 65) height_feedback_flags |= 1u << (15 + i);
            }
            bool feedback_ok = height_feedback_flags == 0;
            bool was_limited = s_height.limited, was_fault = s_height.fault;
            rt_height_apply(&s_height, q, measured, feedback_ok);
            s_j[1].cmd_pos = q[0];
            s_j[2].cmd_pos = q[1];
            if (s_height.limited && !was_limited)
                printf("[HEIGHT] rise blocked near %.1f mm\n", (double)s_height.measured_mm);
            if (s_height.fault && !was_fault && now - t_height_warning >= 500000) {
                t_height_warning = now;
                printf("[HEIGHT] HOLD due to unhealthy feedback/link flags=0x%08lx\n",
                       (unsigned long)height_feedback_flags);
            }
        }
        if (s_level.enabled) {
            float q[3] = {s_j[1].cmd_pos, s_j[2].cmd_pos, s_j[3].cmd_pos};
            bool was_limited = s_level.limited;
            bool feedback_healthy = true;
            for (int i = 1; i <= 3; ++i)
                if (!s_j[i].meas_ok || s_joint_read_us[i] == 0 ||
                    now - s_joint_read_us[i] > 250000 || s_joint_temp[i] >= 65)
                    feedback_healthy = false;
            if (s_level.measured_source && feedback_healthy) {
                float measured[2] = {(float)s_j[1].meas, (float)s_j[2].meas};
                feedback_healthy = rt_level_update_measurement(&s_level, measured, dt_ms);
            }
            rt_level_apply(&s_level, q, dt_ms,
                           have_ctrl && link_ok && s_rail_ok && feedback_healthy &&
                           !s_estop_latched && s_torque_on);
            for (int i = 0; i < 3; ++i) s_j[i + 1].cmd_pos = q[i];
            if (s_height.enabled) rt_height_commit(&s_height, (float[]){q[0], q[1]});
            if (s_level.limited && !was_limited && now - t_level_warning >= 500000) {
                t_level_warning = now;
                printf("[LEVEL] coupled move blocked by range; reverse shoulder/elbow or OFF\n");
            }
        }
        if(s_service_hold) {
            if(!s_torque_on)for(int i=0;i<RC_NJ;i++)
                if(s_j[i].meas_ok&&s_joint_read_us[i]>0&&now-s_joint_read_us[i]<250000)
                    s_service_pose[i]=s_j[i].meas;
            if(s_service_open)s_service_pose[RC_GRIP_CH]=s_j[RC_GRIP_CH].cmd_pos;
            for(int i=0;i<RC_NJ;i++) {
                s_j[i].cmd_pos=s_service_pose[i];s_j[i].rate=s_j[i].vel=s_j[i].carry=0;
            }
        }
        if(fpga_pos_selected())fpga_pos_commit((float[]){s_j[0].cmd_pos,s_j[1].cmd_pos,s_j[2].cmd_pos});
        /* LEVEL_END */

        /* ★ 堵转保护（demo 现场保命用）：
         * 如果某一轴的**实测位置**长时间远远跟不上**目标**，说明它顶在硬限位上
         * 或者被卡住了 —— 这时舵机会持续给力矩，电流全变成热，几十秒就能烫坏。
         * 判据（四个条件同时满足才算堵转）：
         *   ① **电源健康**：回读失败率不能高（⚠️ 见下）
         *   ② **该轴读数新鲜**：meas_ok 为真
         *   ③ 目标与实测差 > RC_STALL_TICKS，且持续 > RC_STALL_MS
         *   ④ 期间实测位置几乎没动（< RC_STALL_MOVE tick）—— 排掉"正在正常追赶"
         * 触发后只松**那一个轴**，并明确报出来；其它轴照常工作。
         *
         * ⚠️⚠️ 条件①是 2026-10-02 实测补上的（**误触发过一次**）：
         *   用户拔掉舵机 5V 后，位置读数变成垃圾值（实测 joint5 报 907，其实在 1715），
         *   于是③④同时"满足"，保护把**一个好关节**的力矩松掉了：
         *       [STALL] *** joint 5 stuck: tgt=1715 meas=907 err=808 ... ***
         *   教训：**任何"基于读数的保护"都必须先确认读数本身可信**，
         *   否则断电/丢帧时它会做出比故障本身更危险的动作（无故松力矩 = 臂掉下来）。 */
        bool rail_healthy = s_rail_ok;
        for (int i = 0; i < RC_NJ; i++) {
            /* ★ 夹爪（双态绝对生效时）不做堵转判定：夹住物体 = "误差常驻 + 不动"，
             * 那是**意图**不是故障 —— 在这里松力矩等于当场把件丢了。
             * 顶限位发热由"目标是绝对位置、不会无限积分"结构性排除。 */
            bool monitor=s_torque_on&&rail_healthy&&s_j[i].meas_ok&&
                s_joint_read_us[i]>0&&now-s_joint_read_us[i]<250000&&
                !(i==RC_GRIP_CH&&rc_grip_preserve_hold_error());
            float err = fabsf(s_j[i].cmd_pos - (float)s_j[i].meas);
            if(rc_stall_step(&s_stall_guard[i],monitor,now,s_j[i].cmd_pos,s_j[i].meas,
                RC_STALL_TICKS,RC_STALL_MS,RC_STALL_MOVE)&&!s_stall_latched[i]) {
                        int moved=abs((int)s_j[i].meas-s_stall_guard[i].reference);
                        s_stall_latched[i] = true;
                        printf("[STALL] *** joint %d stuck: cmd=%d meas=%d err=%d tick for %d ms"
                               " (moved only %d) -> releasing THIS joint's torque ***\n",
                               i + 1, (int)(s_j[i].cmd_pos + 0.5f), (int)s_j[i].meas, (int)err,
                               RC_STALL_MS, moved);
                        printf("[STALL] other joints keep working. Fix the mechanical blockage,\n");
                        printf("[STALL] then reset the board (stall latch clears on boot).\n");
                        sbus_lock();
                        sbus_write_u8(SERVO_ID[i], SB_REG_TORQUE_ENABLE, 0);
                        sbus_unlock();
            }
        }

        /* --- 一帧同步写：6 个轴同时到达（"无阶梯感"的关键）--- */
        if (s_torque_on) {
            uint16_t goal[RC_NJ];
            for (int i = 0; i < RC_NJ; i++) {
                /* ★ 增量式：直接下发 cmd_pos，**不做软限位裁剪**。
                 * 撞机械限位时 servo 自己走不动，cmd_pos 与实测的差会变大，
                 * 由下面的堵转保护接管 —— 这就是"不需要限幅也不会转死"的实现。 */
                int v = (int)(s_j[i].cmd_pos + 0.5f);
                if (v < 0) {
                    v = 0;
                }
                if (v > 4095) {
                    v = 4095;
                }
                goal[i] = (uint16_t)v;
            }
            sbus_lock();
#if SB_SYNC_WRITE_ENABLE
            sbus_sync_write_goal(SERVO_ID, RC_NJ, goal, GOAL_ACC, GOAL_SPEED);
#else
            for (int i = 0; i < RC_NJ; i++) {
                sbus_write_u16(SERVO_ID[i], SB_REG_GOAL_POSITION, goal[i]);
                esp_rom_delay_us(SB_INTERFRAME_US);
            }
#endif
            sbus_unlock();
        }

        /* --- 遥测回传 50 Hz --- */
        if (now - t_tel >= 20000) {
            t_tel = now;
            send_telemetry();
        }

        /* --- 控制周期统计 --- */
        rate_total++;
        if (now - t_draw >= 1000000) {
            rate_ok = rate_total;
            rate_total = 0;
            t_draw = now;
        }

        /* --- 状态日志 1 Hz（报告 §5 的数据就从这里抄）--- */
        if (now - t_log >= 1000000) {
            t_log = now;
            uint32_t tot = s_rx_ok + s_rx_lost;
            int loss_ppm = (tot > 0) ? (int)(s_rx_lost * 1000u / tot) : 0;
            char age_txt[24];
            link_age_text(age_us, age_txt, sizeof age_txt);
            /* 串口没人读时 printf 会把 200 Hz 主循环堵过 50 ms，进而误判
             * control loop delayed；主机没开串口就跳过这一行状态日志。 */
            if (fpga_console_connected())
            printf("[RX] %dHz | link=%s mode=%d age=%5sms loss=%d.%d%% ok=%lu | torque=%d bus=%d "
                   "estop=%d ramp=%d | meas %4d %4d %4d %4d %4d %4d | tgt %4d %4d %4d %4d %4d "
                   "%4d | cmd %4d %4d %4d %4d %4d %4d | %.1fV %dC rdfail=%lu idmis=%lu\n",
                   rate_ok, link_ok ? "OK" : "LOST", s_mode, age_txt,
                   loss_ppm / 10, loss_ppm % 10, (unsigned long)s_rx_ok, (int)s_torque_on,
                   (int)s_bus_ok, (int)s_estop_latched, (int)0, s_j[0].meas,
                   s_j[1].meas, s_j[2].meas, s_j[3].meas, s_j[4].meas, s_j[5].meas,
                   (int)(s_j[0].cmd_pos + 0.5f), (int)(s_j[1].cmd_pos + 0.5f), (int)(s_j[2].cmd_pos + 0.5f),
                   (int)(s_j[3].cmd_pos + 0.5f), (int)(s_j[4].cmd_pos + 0.5f), (int)(s_j[5].cmd_pos + 0.5f),
                   (int)(s_j[0].cmd_pos + 0.5f), (int)(s_j[1].cmd_pos + 0.5f), (int)(s_j[2].cmd_pos + 0.5f),
                   (int)(s_j[3].cmd_pos + 0.5f), (int)(s_j[4].cmd_pos + 0.5f), (int)(s_j[5].cmd_pos + 0.5f),
                   s_vbus_mv / 1000.0, (int)s_temp_max,
                   (unsigned long)(s_read_ok + s_read_fail > 0
                                       ? (s_read_fail * 1000 / (s_read_ok + s_read_fail))
                                       : 0),
                   (unsigned long)sbus_id_mismatches());
        }

        /* --- 严格 5 ms 周期（⚠️ configTICK_RATE_HZ=100 → vTaskDelay 粒度 10 ms，
         *     控不住 5 ms，必须用 esp_rom_delay_us）--- */
        int wait = RC_PERIOD_MS * 1000 - (int)(esp_timer_get_time() - t_loop);
        if (wait > 200) {
            esp_rom_delay_us(wait);
        }
    }
}
