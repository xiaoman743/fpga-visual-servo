# FPGA Deterministic Visual Servoing · 代码归档

> 全国大学生 FPGA 创新设计竞赛参赛作品代码包。
> **本仓库不含学校名称、指导老师姓名、团队成员姓名。**

## 结构

| 目录 | 内容 |
|---|---|
| `fpga-rtl/` | FPGA 工程 RTL（Vivado 2020.2，XC7A100T） |
| `esp32/` | ESP32-S3 执行端固件（ESP-IDF v6.0.1） |
| `sim-p5/` | 延迟对照实验（Vivado xsim，571 个用例） |

详细说明见 `README-代码说明.md`（含**自研/例程来源声明**、编译命令、关键参数、已知限制）。

## 关键点

- FPGA 只输出像素级增量，**不接触舵机**；ESP32 独占全部执行与安全联锁
- 视觉伺服采用图像空间极坐标控制律（角度通道 `cross(u,e)`、径向通道 `|u|²`）
- 延迟对照实验：固定延迟在约 5~6 帧处失稳（`K·τ ≈ π/2`），同均值抖动 183 例无一形成极限环

## 构建

```powershell
# ESP32
idf.py -B build build
idf.py -p <PORT> flash

# 仿真
cd sim-p5
powershell -ExecutionPolicy Bypass -File run_sweeps.ps1
```
