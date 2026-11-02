# 代码包说明 · FPGA 确定性视觉伺服

> 提交用途：作品代码归档。**本包不含任何学校名称、指导老师姓名、团队成员姓名或团队编号。**
> 整理日期：2026-11-03

---

## 一、目录结构

```
代码包/
├── fpga-rtl/          FPGA 工程 RTL（Vivado 2020.2，器件 xc7a100tfgg484-2）
│   ├── vision_*.v     视觉流水线：颜色判决 → 双质心 → 目标锁定 → 径向内缩 → 极坐标伺服 → V2 帧
│   ├── servo_polar.v  极坐标伺服（角度通道 cross + 径向通道 |u|²）
│   ├── target_gate.v  目标锁定（迟滞 + 重捕获）
│   ├── vision_goal.v  目标点沿极径内缩（夹爪占用补偿）
│   ├── delay_inject.v 延迟注入（定值 FIFO + LFSR 抖动，P5 实验用）
│   ├── cmd_parse.v / vision_uart.v / uart_rx.v / uart_tx.v   串口配置与 V2 帧组帧
│   ├── ov5640_*.v / ddr3_*.v / i2c_*.v / lcd_*.v             **原厂例程骨架，未改核心**
│   └── tb_*.v         自检 testbench（**不要加进综合**）
│
├── esp32/             ESP32-S3 执行端固件（ESP-IDF v6.0.1）
│   ├── main/          主循环、舵机总线、位置式控制、水平/高度约束、一键状态机、局部雅可比辨识
│   ├── CMakeLists.txt
│   └── sdkconfig.defaults
│
└── sim-p5/            P5 延迟对照实验（Vivado 2020.2 xsim）
    ├── tb_p5_sweep.v      闭环扫描 tb（伺服 + 延迟注入 + 简化 plant）
    ├── results.csv        A 组 / B 组 / K 矩阵，369 用例
    ├── sigma_sweep.csv    σ 扫描，202 用例
    ├── run_sweeps.ps1     批量运行（用例级断点续跑）
    └── 汇总.md / sigma_scan.md   结果说明与图
```

---

## 二、代码来源声明（评审相关，如实标注）

| 模块 | 来源 |
|---|---|
| `ov5640_dri.v`、`i2c_dri.v`、`i2c_ov5640_rgb565_cfg.v`、`cmos_capture_data.v`、`ddr3_*.v`、`lcd_driver.v`、`lcd_rgb_top.v`、`picture_size.v`、`rd_id.v`、`clk_div.v` | **原厂/开发板例程骨架**，未修改核心逻辑 |
| `vision_features.v`、`rgb_color_mask.v`、`centroid_acc.v`、`target_gate.v`、`vision_goal.v`、`servo_polar.v`、`vision_uart.v`、`vision_panel.v`、`vision_config.v`、`osd_cross.v`、`delay_inject.v`、`cmd_parse.v`、`uart_rx.v`、`uart_tx.v` | **自研** |
| ESP32 侧全部 C 代码（`rt_rx_main.c`、`fpga_link.c`、`fpga_pos.c`、`vision_*.c`、`rt_level.c`、`rt_height.c`、`rt_control.c`、`rt_servo_bus.c` 等） | **自研**（舵机 SCS 协议按公开寄存器表实现） |
| `sim-p5/` 全部 | **自研**（tb 中的 plant 为一阶图像空间模型，非机械臂实测） |

---

## 三、编译与运行

### FPGA（Vivado 2020.2）
```tcl
# 新建工程 → 加入 fpga-rtl/ 下除 tb_*.v 之外的全部 .v
# 器件 xc7a100tfgg484-2；引脚约束参考工程 pin.xdc
# 顶层：ov5640_lcd
```
> ⚠️ `tb_*.v` 仅用于仿真，**不要加入综合源**。

### ESP32-S3（ESP-IDF v6.0.1）
```powershell
idf.py -B build build
idf.py -p <PORT> -b 460800 flash
```
> ⚠️ 工程路径**不要含中文**（CMake/Ninja 会失败），请先复制到 ASCII 路径再编译。

### P5 仿真（xsim）
```powershell
cd sim-p5
powershell -ExecutionPolicy Bypass -File run_sweeps.ps1
```

---

## 四、关键设计参数（便于复现）

| 参数 | 值 |
|---|---|
| 视觉帧率 | 26~27 fps |
| V2 帧 | 59 字节（`@` + 56 hex + CRLF），115200 8N1，CRC16-CCITT |
| 图像极心 | (409, −165) px |
| 伺服增益移位 | `SHIFT_PAN=15`、`SHIFT_RCH=16`（四舍五入） |
| 输出限幅 / 死区 / 保持 | ±4 tick/帧；6 px；连续 8 帧进死区 |
| 目标锁定 | `LOCK_R=24 px`、`FAR_N=4`、`LOST_N=3` |
| 位置式领先窗口 | 150 tick |
| 关节固定区间 | CH2 1400..3200、CH3 1500..3200（∩ EEPROM ±40 tick） |
| 桌面高度模型 | `off2=3043, off3=1011, sign=−1`，桌面原点 87.48446 mm |

---

## 五、已知限制（如实）

1. `DONE` 只表示指令序列完成，**不验证物体是否真的被夹住**（需目视）。
2. 局部图像雅可比依赖一条注入的先验样本启动（样本存 RAM，重启清空）。
3. 仿真 plant 为**纯积分**，无连杆惯性与舵机死区，因此仿真稳定边界比实物**更乐观**。
4. C 组（PC 软件栈）对照实验未完成。
