/*
 * rt_servo_bus.c -- STS3215 总线舵机驱动（FEETECH SCS 协议，UART 1 Mbps）
 * ============================================================================
 * 这一层是**已实测通过**的部分，别改接线，也别改协议格式：
 *   接线（与 D:\Code\urt2_probe_idf 实测一致，⛔ 不是 A/B/V2/G 那套 RS485）：
 *     舵机板 TTL 排针 `TX` → ESP32-S3 GPIO18 (RX)
 *     舵机板 TTL 排针 `RX` → 【983 Ω】 → ESP32-S3 GPIO17 (TX)
 *     GND ↔ GND；板上 `5V` 与 `DTR` 都**不接**
 *   实测：6/6 全应答、Model 全 = 777、读成功率 97.5%→100%、写 Δ=99 tick。
 *
 * ★ 为什么 200 Hz 控制循环里必须用**同步写（SYNC WRITE，ID=0xFE 广播）**：
 *   单轴逐条写 6 个舵机 = 6 条帧，1 Mbps 下每条约 0.2 ms 加上帧间隔，
 *   一个 5 ms 周期里光发指令就吃掉 1.5–3 ms；再加上回读就必然超时。
 *   同步写**一帧搞定 6 个轴**，把写侧开销压到 0.2 ms 量级 —— 这才是
 *   "200 Hz 更新 + 无阶梯感"能同时成立的物理前提。
 *   （逐轴写仍保留为回退路径：SYNC_WRITE_ENABLE=0 时就退回逐轴写。）
 * ============================================================================
 */
#include "rt_servo_bus.h"
#include "servo_response.h"

#include <string.h>

#include "driver/uart.h"
#include "driver/gpio_filter.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/* ------------------------------------------------------------------ SCS 协议 */
static uint8_t scs_checksum(const uint8_t *body, int n)
{
    unsigned sum = 0;
    for (int i = 0; i < n; i++) {
        sum += body[i];
    }
    return (uint8_t)(~(sum & 0xFF));
}

/* 组帧：FF FF ID LEN INSTR PARAMS... CHECK ；LEN = 参数个数 + 2 */
static int scs_build(uint8_t id, uint8_t instr, const uint8_t *params, int nparams, uint8_t *out)
{
    int k = 0;
    out[k++] = 0xFF;
    out[k++] = 0xFF;
    out[k++] = id;
    out[k++] = (uint8_t)(nparams + 2);
    out[k++] = instr;
    for (int i = 0; i < nparams; i++) {
        out[k++] = params[i];
    }
    out[k] = scs_checksum(&out[2], 3 + nparams);
    return k + 1;
}

static void bus_drain_input(void)
{
    uint8_t tmp[64];
    int n;
    /* timeout=0 = 非阻塞，把缓冲里残留的都倒掉。
     * 这很关键：**不排空就会出现"迟到的回帧被当成本次应答"** ——
     * 于是遥测里的关节角是上一次请求的值，症状是"位置显示慢一拍/跳着走"，
     * 而实测时很难看出来（值都是合理的）。 */
    while ((n = uart_read_bytes(SB_UART, tmp, sizeof(tmp), 0)) > 0) {
        /* keep draining */
        (void)n;
    }
}

/* ★ 原始字节计数器（诊断用）。
 * 2026-10-02 实测教训：排查"扫不到舵机"时，"**一个字节都没收到**"和
 * "**收到字节但解析不出**"是两个完全不同的故障 —— 前者是线/UART 配置，
 * 后者是波特率/协议。只看 NO ANSWER 分不清，于是白查了很久。
 * 所以在这里统计：每次读帧超时时，把"本次事务期间收到的原始字节数"加上去。
 * 读出来是 0 → 物理层没通；读出来很大 → 通的，只是解析/波特率不对。 */
static volatile uint32_t s_raw_bytes;      /* 累计收到的原始字节 */
static volatile uint32_t s_raw_bytes_win;  /* 本窗口（供打印后清零） */
static volatile uint32_t s_txn_timeouts;   /* 超时次数 */
/* ★ 2026-10-11 修复"静默串位"：回帧校验和合法但 ID 不是本事务请求的舵机
 * （上一条事务的迟到应答窜进了本窗口）→ 拒收并计数。修复前这种帧被照单
 * 全收，A 轴的位置被记到 B 轴头上，遥测/示教录制悄悄变垃圾。 */
static volatile uint32_t s_id_mismatch;    /* ID 失配拒收次数 */
static int64_t s_device_error_report_us[6];
/* ★ 原始字节抓包：把最近收到的字节按 hex 存下来供打印。
 * 2026-10-02 排查"扫不到舵机"时，就是靠它把"0 字节"和"6 字节但校验不过"
 * 分开的 —— 前者物理层，后者波特率/边沿质量。只看 NO ANSWER 分不出来。 */
static uint8_t s_raw_hex[32];
static volatile int s_raw_hex_n;

void sbus_raw_hex_dump(void)
{
    printf("[BUS] raw hex (%d bytes):", s_raw_hex_n);
    for (int i = 0; i < s_raw_hex_n; i++) {
        printf(" %02X", s_raw_hex[i]);
    }
    printf("\n");
    if (s_raw_hex_n > 0) {
        /* 一个合法的 SCS 应答应该是 FF FF ID LEN ERR CHECK... */
        if (s_raw_hex[0] == 0xFF && s_raw_hex_n >= 3 && s_raw_hex[1] == 0xFF) {
            printf("[BUS] -> looks like a frame START (FF FF) but the payload/checksum is bad:\n");
            printf("[BUS]    baud mismatch or signal-integrity problem on the RX path\n");
        } else {
            printf("[BUS] -> does NOT even start with FF FF: bytes are mangled\n");
            printf("[BUS]    (wrong baud, floating bus, or RX not driven)\n");
        }
    }
    s_raw_hex_n = 0;
}

uint32_t sbus_raw_bytes_take(void)
{
    uint32_t v = s_raw_bytes_win;
    s_raw_bytes_win = 0;
    return v;
}
uint32_t sbus_txn_timeouts(void) { return s_txn_timeouts; }
uint32_t sbus_id_mismatches(void) { return s_id_mismatch; }

/* 收一帧：跳过噪声，只在 FF FF 之后认帧，并校验 CHECK。
 * ★ buf/len 用调用者给的缓冲：原来的探针固件用栈上 64 字节，
 *   这里保留同样大小（SCS 帧最长也就十几字节）。 */
/* want_id != 0 时校验回帧 ID：不匹配 = 上一事务的迟到应答窜窗 → 丢弃，
 * 并在剩余窗口内继续扫（真正的应答可能紧随其后到达）。 */
static bool scs_read_frame(uint8_t want_id, unsigned expected, uint8_t *err,
                           uint8_t *payload, int *plen, int timeout_ms)
{
    servo_response_t parser={0};
    int64_t deadline=esp_timer_get_time()+(int64_t)timeout_ms*1000;
    while(esp_timer_get_time()<deadline) {
        uint8_t byte;
        if(uart_read_bytes(SB_UART,&byte,1,0)!=1){esp_rom_delay_us(20);continue;}
        s_raw_bytes++;s_raw_bytes_win++;
        if(s_raw_hex_n<(int)sizeof(s_raw_hex))s_raw_hex[s_raw_hex_n++]=byte;
        unsigned before=parser.wrong_id;
        bool ok=servo_response_feed(&parser,byte,want_id,expected,payload,err);
        s_id_mismatch+=parser.wrong_id-before;
        if(parser.device_errors&&want_id>=1&&want_id<=6) {
            int64_t now=esp_timer_get_time();
            if(!s_device_error_report_us[want_id-1]||now-s_device_error_report_us[want_id-1]>=5000000) {
                s_device_error_report_us[want_id-1]=now;
                printf("[BUSFAULT] ID=%u error=0x%02X checksum/id/length valid; feedback rejected\n",want_id,parser.last_error);
            }
        }
        if(ok){if(plen)*plen=(int)expected;return true;}
    }
    s_txn_timeouts++;
    if(s_txn_timeouts<=8)printf("[BUSDBG] timeout id=%u expected=%u wrong_id=%u rejected=%u buffered=%u\n",
        want_id,expected,parser.wrong_id,parser.rejected,parser.used);
    return false;
}

/* 发一条指令并等应答。返回 false = 超时/校验失败。
 * timeout_ms 显式传入：**控制回路里必须用短超时**（SB_READ_TIMEOUT_SHORT_MS），
 * 否则一次丢帧就阻塞 25 ms，6 个轴轮一遍能把 5 ms 周期拖到 100 ms 以上
 * —— 实测会把行首频率从 200 Hz 打到 46 Hz，现场表现是"推了没反应、一顿一顿"。 */
static bool scs_txn_t(uart_port_t uart, uint8_t id, uint8_t instr, const uint8_t *params,
                      int nparams, uint8_t *err, uint8_t *payload, int *plen, int timeout_ms)
{
    (void)uart;
    if (timeout_ms <= 0) {
        timeout_ms = SB_READ_TIMEOUT_MS;
    }
    uint8_t frame[32];
    int n = scs_build(id, instr, params, nparams, frame);
    uart_flush_input(SB_UART);
    uart_write_bytes(SB_UART, (const char *)frame, n);
    uart_wait_tx_done(SB_UART, pdMS_TO_TICKS(20));
    esp_rom_delay_us(SB_INTERFRAME_US);
    unsigned expected=(instr==SCS_INST_READ&&nparams==2)?params[1]:0;
    return scs_read_frame(id, expected, err, payload, plen, timeout_ms);
}

static bool scs_txn(uint8_t id, uint8_t instr, const uint8_t *params, int nparams, uint8_t *err,
                    uint8_t *payload, int *plen)
{
    return scs_txn_t(SB_UART, id, instr, params, nparams, err, payload, plen, SB_READ_TIMEOUT_MS);
}

/* ------------------------------------------------------------------ 公开接口 */
static SemaphoreHandle_t s_bus_mtx;
static bool s_inited;
static gpio_glitch_filter_handle_t s_bus_rx_filter;

void sbus_init(void)
{
    const uart_config_t cfg = {
        .baud_rate = SB_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_XTAL,
    };
    ESP_ERROR_CHECK(uart_driver_install(SB_UART, SB_RX_BUF, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(SB_UART, &cfg));
    ESP_ERROR_CHECK(
        uart_set_pin(SB_UART, SB_PIN_TX, SB_PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    const gpio_pin_glitch_filter_config_t filter_config={
        .gpio_num=SB_PIN_RX,.clk_src=GLITCH_FILTER_CLK_SRC_DEFAULT};
    ESP_ERROR_CHECK(gpio_new_pin_glitch_filter(&filter_config,&s_bus_rx_filter));
    ESP_ERROR_CHECK(gpio_glitch_filter_enable(s_bus_rx_filter));
    uart_set_rx_full_threshold(SB_UART,1);
    uart_set_rx_timeout(SB_UART,2);
    uart_flush_input(SB_UART);
    s_bus_mtx = xSemaphoreCreateMutex();
    s_inited = true;
}

void sbus_lock(void)
{
    if (s_bus_mtx) {
        xSemaphoreTake(s_bus_mtx, portMAX_DELAY);
    }
}

void sbus_unlock(void)
{
    if (s_bus_mtx) {
        xSemaphoreGive(s_bus_mtx);
    }
}

bool sbus_ping(uint8_t id, uint8_t *err)
{
    return scs_txn(id, SCS_INST_PING, NULL, 0, err, NULL, NULL);
}

bool sbus_read_reg(uint8_t id, uint8_t addr, uint8_t len, uint8_t *out)
{
    if (len == 0 || len > 8) {
        return false;
    }
    uint8_t p[2] = {addr, len};
    uint8_t err = 0;
    uint8_t pl[8];
    int plen = 0;
    if (!scs_txn(id, SCS_INST_READ, p, 2, &err, pl, &plen)) {
        return false;
    }
    if (plen < len) {
        return false;
    }
    memcpy(out, pl, len);
    return true;
}

bool sbus_write_reg(uint8_t id, uint8_t addr, const uint8_t *data, int len)
{
    uint8_t p[10];
    if (len <= 0 || len > 8) {
        return false;
    }
    p[0] = addr;
    memcpy(&p[1], data, len);
    uint8_t err = 0;
    return scs_txn(id, SCS_INST_WRITE, p, len + 1, &err, NULL, NULL);
}

bool sbus_read_u16(uint8_t id, uint8_t addr, uint16_t *out)
{
    uint8_t b[2];
    if (!sbus_read_reg(id, addr, 2, b)) {
        return false;
    }
    *out = (uint16_t)(b[0] | (b[1] << 8));
    return true;
}

bool sbus_read_u8(uint8_t id, uint8_t addr, uint8_t *out)
{
    return sbus_read_reg(id, addr, 1, out);
}

bool sbus_write_u16(uint8_t id, uint8_t addr, uint16_t v)
{
    uint8_t b[2] = {(uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF)};
    return sbus_write_reg(id, addr, b, 2);
}

bool sbus_write_u8(uint8_t id, uint8_t addr, uint8_t v) { return sbus_write_reg(id, addr, &v, 1); }

/* 读一个轴的**状态块**：一条 READ(56, 8) 全拿到。
 * 这条替代了原来的 sbus_read_axis()（那时位置/电压/温度分 3 次读）——
 * 事务数从 18/周期降到 6/周期，最坏耗时降到 1/3，是守住 200 Hz 的关键。 */
bool sbus_read_status(uint8_t servo_id, uint16_t *pos, uint16_t *speed, uint16_t *load,
                      uint8_t *volt, uint8_t *temp, int timeout_ms)
{
    uint8_t p[2] = {SB_REG_PRESENT_POSITION, 8}; /* 56..63 */
    uint8_t err = 0;
    uint8_t pl[8];
    int plen = 0;
    if (!scs_txn_t(SB_UART, servo_id, SCS_INST_READ, p, 2, &err, pl, &plen, timeout_ms)) {
        return false;
    }
    if (plen != 8 || (unsigned)(pl[0]|(pl[1]<<8))>4095 || pl[7]>=100 || pl[6]<30 || pl[6]>90) {
        return false;
    }
    if (pos) {
        *pos = (uint16_t)(pl[0] | (pl[1] << 8)); /* 56 Present_Position */
    }
    if (speed) {
        *speed = (uint16_t)(pl[2] | (pl[3] << 8)); /* 58 Present_Speed */
    }
    if (load) {
        *load = (uint16_t)(pl[4] | (pl[5] << 8)); /* 60 Present_Load */
    }
    if (volt) {
        *volt = pl[6]; /* 62 Present_Voltage（0.1 V） */
    }
    if (temp) {
        *temp = pl[7]; /* 63 Present_Temperature（℃） */
    }
    return true;
}

/* 6 个轴一起开/关力矩。关力矩前必须确认臂已被托住 —— 否则会因重力掉下来 */
void sbus_torque_all(const uint8_t *ids, int n, bool on)
{
    uint8_t v = on ? 1 : 0;
    for (int i = 0; i < n; i++) {
        if (!sbus_write_u8(ids[i], SB_REG_TORQUE_ENABLE, v)) {
            ESP_LOGW("sbus", "torque %s failed on ID %d", on ? "ON" : "OFF", ids[i]);
        }
        esp_rom_delay_us(SB_INTERFRAME_US);
    }
}

/* 同步写一帧（广播 ID 0xFE），一次性更新 n 个轴的目标位置。
 * 帧格式（FEETECH SYNC WRITE，与官方 SCS SDK 的 syncWritePosEx 一致）：
 *   [0x29 0x07]                     起始地址 = 加速度(41)、每轴 **7** 字节
 *   per axis: [id acc(1) pos(2) time(2) speed(2)]
 *   [CHECK]
 * ★★ 2026-10-02 深夜实测 bug：这里原来写的是"每轴 6 字节"，而实际打包了 7 字节。
 *    舵机按"ID+6字节"切块 → 第 1 个轴之后全部错位（speed 高字节被当成下一个 ID），
 *    而广播同步写**没有应答** → 静默失败。现场表现："通道值在变、cmd_pos 在涨、
 *    实测位置纹丝不动、堵转也不报"。改成 7（1+2+2+2）后 6 轴全部跟动。 */
bool sbus_sync_write_goal(const uint8_t *ids, int n, const uint16_t *pos, uint8_t acc,
                          uint16_t speed)
{
    uint8_t params[2 + SB_SYNC_MAX * 8];
    int k = 0;
    params[k++] = SB_REG_ACCELERATION; /* 起始地址 = 41 */
    params[k++] = 7;                   /* 每轴 7 字节：acc(1)+pos(2)+time(2)+speed(2) */
    for (int i = 0; i < n && i < SB_SYNC_MAX; i++) {
        params[k++] = ids[i];
        params[k++] = acc;
        params[k++] = (uint8_t)(pos[i] & 0xFF);
        params[k++] = (uint8_t)((pos[i] >> 8) & 0xFF);
        params[k++] = 0; /* time = 0：不用舵机内部的时间插值，位置由我们每周期给 */
        params[k++] = 0;
        params[k++] = (uint8_t)(speed & 0xFF);
        params[k++] = (uint8_t)((speed >> 8) & 0xFF);
    }
    uint8_t frame[64];
    int nf = scs_build(SCS_BROADCAST_ID, SCS_INST_SYNC_WRITE, params, k, frame);
    bus_drain_input(); /* 广播没有应答，但要把可能残留的读回帧倒掉 */
    uart_write_bytes(SB_UART, (const char *)frame, nf);
    return uart_wait_tx_done(SB_UART, pdMS_TO_TICKS(20)) == ESP_OK;
}

bool sbus_is_inited(void) { return s_inited; }
