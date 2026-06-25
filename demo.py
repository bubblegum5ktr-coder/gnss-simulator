"""
GNSS 设备模拟器 — 演示脚本
=========================
逐步展示模拟器的各种功能：
  1. 静止定位 (RTK固定解)
  2. 直线运动 (行人步行)
  3. 圆周运动 + 故障注入 (信号丢失)
  4. 多星座输出 (GPS + 北斗)
"""
import time
import socket
import threading
from simulator import GnssSimulator, FaultConfig
from trajectory import StaticPosition, LinearMotion, CircleMotion


def print_section(title):
    print(f"\n{'=' * 55}")
    print(f"  {title}")
    print(f"{'=' * 55}")


def receive_samples(port, duration=3, label="NMEA 数据"):
    """连接模拟器并接收几秒数据用于展示"""
    lines = []
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(duration + 2)
    try:
        sock.connect(('localhost', port))
        start = time.time()
        while time.time() - start < duration:
            try:
                data = sock.recv(4096)
                if data:
                    for line in data.decode('ascii').strip().split('\r\n'):
                        if line and line not in lines[-5:]:
                            lines.append(line)
            except socket.timeout:
                break
    finally:
        sock.close()
    return lines


# ════════════════════════════════════════════════════════
#  场景 1: 静止定位
# ════════════════════════════════════════════════════════
print_section("1. 静止定位 — RTK固定解, 22.5°N 113.0°E")
sim1 = GnssSimulator(
    trajectory=StaticPosition(lat=22.5431, lon=113.9408, altitude=50.0),
    frequency=1,
    talkers=['GP'],
)
sim1.start(port=5001)
time.sleep(0.5)
lines = receive_samples(5001, 2)
sim1.stop()

for line in lines[:5]:
    print(f"  {line}")
print(f"  ... (共 {len(lines)} 条语句)")
print(f"  统计: {sim1.stats.epochs_sent} 个历元, "
      f"{sim1.stats.sentences_sent} 条语句")

# ════════════════════════════════════════════════════════
#  场景 2: 直线运动
# ════════════════════════════════════════════════════════
print_section("2. 直线运动 — 步行速度 1.4m/s, 向东北方向")
sim2 = GnssSimulator(
    trajectory=LinearMotion(start_lat=22.5431, start_lon=113.9408,
                            speed_mps=1.4, bearing_deg=45),
    frequency=5,  # 5Hz 高频
    talkers=['GP'],
)
sim2.start(port=5002)
time.sleep(0.5)
lines = receive_samples(5002, 1.5)
sim2.stop()

# 取两个不同时间的 GGA 对比坐标变化
ggas = [l for l in lines if 'GGA' in l]
if len(ggas) >= 2:
    print(f"  第1条 GGA: {ggas[0][:60]}...")
    print(f"  最后 GGA:  {ggas[-1][:60]}...")
    print(f"  → 坐标在变化 (直线运动)")
    print(f"  频率 5Hz: {sim2.stats.epochs_sent} 个历元 / 1.5秒")

# ════════════════════════════════════════════════════════
#  场景 3: 圆周运动 + 故障注入
# ════════════════════════════════════════════════════════
print_section("3. 圆周运动 + 故障注入 — 信号丢失模拟")
fc = FaultConfig(
    fix_degradation_prob=0.3,    # 30% 概率定位质量跳变
    satellite_drop_prob=0.2,     # 20% 概率卫星数骤降
    checksum_error_rate=0.1,     # 10% 概率校验和错误
)
sim3 = GnssSimulator(
    trajectory=CircleMotion(radius_m=200, speed_mps=5.0),
    frequency=2,
    talkers=['GP'],
    fault_config=fc,
)
sim3.start(port=5003)
time.sleep(0.5)
lines = receive_samples(5003, 3)
sim3.stop()

# 展示含故障的语句
ggas_3 = [l for l in lines if 'GGA' in l]
for gga in ggas_3[:3]:
    print(f"  {gga}")
print(f"  故障注入: {sim3.stats.faults_injected} 次 (fix跳变/卫星骤降/校验和错误)")
print(f"  共 {sim3.stats.epochs_sent} 个历元")

# ════════════════════════════════════════════════════════
#  场景 4: 多星座 (GPS + 北斗)
# ════════════════════════════════════════════════════════
print_section("4. 多星座 — GPS + 北斗 (GP + BD)")
sim4 = GnssSimulator(
    trajectory=StaticPosition(lat=22.5431, lon=113.9408),
    frequency=1,
    talkers=['GP', 'BD'],  # GPS + 北斗
)
sim4.start(port=5004)
time.sleep(0.5)
lines = receive_samples(5004, 1.5)
sim4.stop()

for line in lines:
    # 高亮不同星座
    marker = ""
    if 'GPGSV' in line:
        marker = " ← GPS 可见星"
    elif 'BDGSV' in line:
        marker = " ← 北斗可见星"
    elif 'GPGSA' in line:
        marker = " ← GPS 精度因子"
    elif 'BDGSA' in line:
        marker = " ← 北斗精度因子"
    elif 'GNGGA' in line:
        marker = " ← 多系统合并解"
    print(f"  {line}{marker}")

print(f"\n  一个历元共 {len(lines)} 条语句 (GPS + 北斗)")

# ════════════════════════════════════════════════════════
#  总结
# ════════════════════════════════════════════════════════
print_section("总结")
print(f"""
  4 个场景全部演示完成:
    ✓ 静止定位 → 测试基础输出
    ✓ 直线运动 → 模拟行人/车辆
    ✓ 圆周+故障 → 信号丢失/校验和错误
    ✓ 多星座    → GPS + 北斗同时输出

  模拟器用法:
    from simulator import GnssSimulator
    sim = GnssSimulator(trajectory=..., frequency=5, talkers=['GP','BD'])
    sim.start(port=5000)
    # 用 telnet localhost 5000 或 nmea-validator 连接
    sim.stop()
""")
