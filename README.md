# GNSS Device Simulator · GNSS 设备模拟器

A TCP-based RTK receiver simulator that streams NMEA-0183 positioning data — designed for testing GNSS software without real hardware.

基于 TCP 的 RTK 接收机模拟器，持续发送 NMEA-0183 定位数据 —— 无需真实设备即可测试 GNSS 上层软件。

## Features · 功能

- **4 trajectory modes**: static positioning, linear motion, circular motion, random walk
- **Multi-constellation**: GPS, BeiDou, GLONASS, Galileo (single or combined)
- **Configurable frequency**: 1 Hz / 5 Hz / 10 Hz (simulate low-end to high-end receivers)
- **Fault injection engine**: packet loss, latency, position noise, fix-quality degradation (4→5→1→0), satellite dropout, checksum corruption
- **TCP server**: multiple clients can connect simultaneously
- **Receiver script**: capture data to file, or pipe directly to [nmea-validator](https://github.com/your-account/nmea-validator) for validation

---

- **4 种轨迹**: 静止定位、匀速直线、圆周运动、随机走动
- **多星座支持**: GPS、北斗、GLONASS、Galileo（单系统或任意组合）
- **频率可调**: 1 Hz / 5 Hz / 10 Hz（模拟低端到高端接收机）
- **故障注入引擎**: 丢包、延迟、坐标噪声、定位质量跳变、卫星骤降、校验和错误
- **TCP 服务器**: 支持多客户端同时连接
- **接收脚本**: 采集数据保存文件，或对接 [nmea-validator](https://github.com/your-account/nmea-validator) 自动校验

## Quick Start · 快速开始

```bash
# 1. Start the simulator (choose a scenario interactively)
python simulator.py

# 2. In another terminal, capture and validate
python receiver.py --output test.nmea --validate --duration 10
```

Or run the guided demo (4 scenarios in sequence):

```bash
python demo.py
```

## Project Structure · 项目结构

```
gnss-simulator/
├── simulator.py         # Core: TCP server, NMEA data stream, fault injection
├── trajectory.py        # Trajectory generators (static / linear / circle / random)
├── nmea_builder.py      # NMEA-0183 sentence builder (GGA/RMC/GSA/GSV/VTG)
├── receiver.py          # TCP client: capture, save, validate
├── demo.py              # Guided demo: 4 scenarios step by step
├── test_simulator.py    # pytest suite (22 tests · 条测试)
├── reports/             # Test report output directory
├── .gitignore
└── README.md
```

## Scenarios · 内置场景

Run `python simulator.py` and choose from:

| # | Scenario · 场景 | Freq | Trajectory · 轨迹 | Faults · 故障 |
|---|----------------|------|-------------------|--------------|
| 0 | Static · 静止定位 | 1 Hz | Fixed point | None |
| 1 | Walking · 直线步行 | 1 Hz | Linear 1.4 m/s | None |
| 2 | Driving · 直线行驶 | 5 Hz | Linear 13.9 m/s | None |
| 3 | Circle · 圆周运动 | 5 Hz | Circle r=200m | None |
| 4 | Random walk · 随机走动 | 1 Hz | Random 2m step | None |
| 5 | Signal loss · 信号丢失 | 5 Hz | Circle | Fix drop / sat drop / checksum err |
| 6 | Multi-GNSS · 多星座 | 1 Hz | Fixed point | None |

## Programmatic Usage · 编程调用

```python
from simulator import GnssSimulator, FaultConfig
from trajectory import CircleMotion

# Circular motion at 5 Hz with faults
sim = GnssSimulator(
    trajectory=CircleMotion(center_lat=22.5431, center_lon=113.9408,
                            radius_m=200, speed_mps=5.0),
    frequency=5,
    talkers=['GP', 'BD'],          # GPS + BeiDou
    fault_config=FaultConfig(
        fix_degradation_prob=0.3,   # 30% chance fix quality drops
        checksum_error_rate=0.1,    # 10% chance corrupted checksum
        position_noise_m=3.0,       # ±3m random position noise
    ),
)
sim.start(port=5000)
# ... connect with receiver or telnet localhost 5000
sim.stop()
```

## Fault Injection · 故障注入

| Parameter · 参数 | Range · 范围 | Description · 说明 |
|------------------|-------------|---------------------|
| `packet_loss_rate` | 0.0–1.0 | Drop entire epochs · 整历元丢弃 |
| `extra_delay_ms` | ≥ 0 | Artificial latency · 人为延迟 |
| `position_noise_m` | ≥ 0 | Gaussian noise on lat/lon · 坐标高斯噪声 |
| `fix_degradation_prob` | 0.0–1.0 | Fix quality 4→5→1→0 degradation |
| `satellite_drop_prob` | 0.0–1.0 | Sudden satellite count drop · 卫星数骤降 |
| `checksum_error_rate` | 0.0–1.0 | Intentional checksum corruption · 校验和故意写错 |

## Receiver Usage · 接收器用法

```bash
# Capture 10 seconds and print preview
python receiver.py --duration 10

# Save to file
python receiver.py --output captured.nmea --duration 30

# Capture + validate with nmea-validator
python receiver.py --output test.nmea --validate --duration 15

# Connect to a remote simulator
python receiver.py --host 192.168.1.100 --port 5000
```

## Integration with nmea-validator · 对接校验工具

This simulator pairs with [nmea-validator](https://github.com/your-account/nmea-validator):

```
[gnss-simulator] --TCP--> [receiver.py] --save--> [*.nmea file] --validate--> [nmea-validator]
```

The `--validate` flag in `receiver.py` calls nmea-validator automatically. Requires nmea-validator to be installed at `D:/workspace/nmea-validator/` (update `sys.path` in `receiver.py` if your path differs).

## Run Tests · 运行测试

```bash
python -m pytest test_simulator.py -v
```

## Requirements · 依赖

- Python 3.10+
- pytest (for running tests)
- [nmea-validator](https://github.com/your-account/nmea-validator) (optional, for `--validate`)

No third-party packages required for core functionality.
