"""
GNSS 设备模拟器
==============
模拟 RTK 接收机，通过 TCP 持续发送 NMEA-0183 定位数据。
支持多种轨迹、多星座、故障注入。

用法:
    # 直接运行 — 默认轨迹, 1Hz, 端口 5000
    python simulator.py

    # 编程方式使用
    from simulator import GnssSimulator
    from trajectory import CircleMotion

    sim = GnssSimulator(
        trajectory=CircleMotion(radius_m=200, speed_mps=5.0),
        frequency=5,
        talkers=['GP', 'BD'],
    )
    sim.start(port=5000)
"""
import math
import random
import socket
import threading
import time
from datetime import datetime, UTC
from dataclasses import dataclass, field

from nmea_builder import build_epoch
from trajectory import (Position, StaticPosition, LinearMotion,
                         CircleMotion, RandomWalk)


@dataclass
class FaultConfig:
    """故障注入配置"""
    packet_loss_rate: float = 0.0       # 丢包率 0.0~1.0
    extra_delay_ms: float = 0.0         # 额外延迟 (毫秒)
    position_noise_m: float = 0.0       # 坐标随机噪声 (米, 标准差)
    fix_degradation_prob: float = 0.0   # 定位质量跳变概率 (每次历元)
    satellite_drop_prob: float = 0.0    # 卫星数骤降概率
    checksum_error_rate: float = 0.0    # 校验和故意写错概率


@dataclass
class SimStats:
    """模拟器运行统计"""
    epochs_sent: int = 0
    bytes_sent: int = 0
    sentences_sent: int = 0
    faults_injected: int = 0
    clients_connected: int = 0
    start_time: float = 0.0
    errors: list = field(default_factory=list)


# ── 默认卫星配置 ──────────────────────────────────────────
def _default_sats_in_view(talkers=('GP', 'BD')) -> list[dict]:
    """生成模拟可见卫星列表"""
    sats = []
    for talker in talkers:
        base_prn = {'GP': 1, 'BD': 30, 'GL': 60, 'GA': 80}[talker]
        for i in range(8):
            sats.append({
                'prn': base_prn + i,
                'talker': talker,
                'elevation': random.randint(10, 85),
                'azimuth': random.randint(0, 359),
                'snr': random.randint(30, 50),
            })
    return sats


def _default_sat_ids(talkers=('GP', 'BD')) -> list[int]:
    """生成活跃卫星编号列表"""
    return [1, 3, 4, 7, 8, 10, 11, 14, 16, 22, 26, 27,
            30, 31, 33, 35, 36, 37]


class GnssSimulator:
    """GNSS 设备模拟器 — TCP server + NMEA 数据流"""

    def __init__(self, trajectory=None, frequency: float = 1.0,
                 talkers=('GP', 'BD'),
                 fault_config: FaultConfig = None):
        """
        trajectory: 轨迹生成器 (默认: StaticPosition)
        frequency: 发送频率 (Hz), 1=1Hz, 5=5Hz, 10=10Hz
        talkers: 星座系统列表, 如 ('GP',) 或 ('GP', 'BD', 'GL')
        fault_config: 故障注入配置, None = 无故障
        """
        self.trajectory = trajectory or StaticPosition()
        self.frequency = frequency
        self.talkers = list(talkers)
        self.fault_config = fault_config or FaultConfig()
        self.stats = SimStats()

        self._server_socket: socket.socket | None = None
        self._clients: list[socket.socket] = []
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None

        # 卫星信息 (可刷新)
        self._sats_in_view = _default_sats_in_view(talkers)
        self._sat_ids = _default_sat_ids(talkers)
        self._epoch_number = 0

    # ── 故障注入逻辑 ──────────────────────────────────────

    def _apply_faults(self, pos: Position) -> tuple[Position, list[str]]:
        """对历元数据应用故障注入, 返回 (修改后的位置, 故障描述列表)"""
        fc = self.fault_config
        faults = []

        # 定位质量降级 (signal loss pattern: 4→5→1→0)
        if random.random() < fc.fix_degradation_prob:
            old = pos.fix_quality
            if pos.fix_quality >= 4:
                pos.fix_quality = random.choice([5, 1, 0])
            elif pos.fix_quality == 5:
                pos.fix_quality = random.choice([1, 0])
            else:
                pos.fix_quality = 0
            faults.append(f"fix_quality: {old}→{pos.fix_quality}")

        # 卫星数骤降
        if random.random() < fc.satellite_drop_prob:
            old = pos.satellites
            pos.satellites = max(4, pos.satellites - random.randint(5, 12))
            pos.hdop = pos.hdop * random.uniform(1.5, 4.0)
            pos.pdop = pos.pdop * random.uniform(1.5, 4.0)
            faults.append(f"satellites: {old}→{pos.satellites}")

        # 坐标噪声
        if fc.position_noise_m > 0:
            noise_lat = random.gauss(0, fc.position_noise_m) / 111320.0
            noise_lon = (random.gauss(0, fc.position_noise_m) /
                         (111320.0 * math.cos(math.radians(pos.lat))))
            pos.lat += noise_lat
            pos.lon += noise_lon
            faults.append(f"position_noise: ±{fc.position_noise_m:.1f}m")

        return pos, faults

    # ── 核心循环 ──────────────────────────────────────────

    def _generate_epoch(self) -> list[str]:
        """生成一个历元的全部 NMEA 语句"""
        pos = self.trajectory.next()

        # 刷新卫星配置 (模拟卫星缓慢变化)
        if self._epoch_number % 30 == 0:
            for sat in self._sats_in_view:
                sat['elevation'] = (sat['elevation'] +
                                    random.randint(-5, 5)) % 90
                sat['azimuth'] = (sat['azimuth'] +
                                  random.randint(-8, 8)) % 360
                sat['snr'] = max(20, min(55,
                                         sat['snr'] + random.randint(-3, 3)))

        # 注入故障
        faults = []
        if self.fault_config:
            pos, faults = self._apply_faults(pos)
            if faults:
                self.stats.faults_injected += len(faults)

        # 填充默认值
        pos.sat_ids = pos.sat_ids or tuple(self._sat_ids)
        pos.sats_in_view = pos.sats_in_view or tuple(self._sats_in_view)

        # 时间
        now = datetime.now(UTC).replace(tzinfo=None)
        utc_time = now.strftime('%H%M%S') + f'.{now.microsecond // 100000}'
        date_str = now.strftime('%d%m%y')

        sentences = build_epoch(
            talkers=self.talkers,
            utc_time=utc_time,
            date_str=date_str,
            lat=pos.lat,
            lon=pos.lon,
            altitude=pos.altitude,
            fix_quality=pos.fix_quality,
            satellites=pos.satellites,
            hdop=pos.hdop,
            pdop=pos.pdop,
            vdop=pos.vdop,
            speed_knots=pos.speed_knots,
            course=pos.course,
            sat_ids=list(pos.sat_ids),
            sats_in_view=list(pos.sats_in_view),
        )

        # 校验和故意写错
        fc = self.fault_config
        if fc and random.random() < fc.checksum_error_rate and sentences:
            idx = random.randint(0, len(sentences) - 1)
            # 把最后一个字符改掉 (破坏校验和)
            s = sentences[idx]
            last = s[-1]
            new_last = '0' if last != '0' else '1'
            sentences[idx] = s[:-1] + new_last
            faults.append(f"checksum_error: sentence[{idx}]")

        self._epoch_number += 1
        return sentences

    def _send_to_all(self, data: bytes):
        """向所有连接的客户端发送数据"""
        with self._lock:
            dead = []
            for client in self._clients:
                try:
                    client.sendall(data)
                except Exception:
                    dead.append(client)
            for d in dead:
                self._clients.remove(d)

    def _broadcast_loop(self):
        """主循环: 按频率生成并发送 NMEA 数据"""
        interval = 1.0 / self.frequency

        while self._running:
            loop_start = time.time()

            # 丢包判断
            fc = self.fault_config
            if fc and random.random() < fc.packet_loss_rate:
                self.stats.epochs_sent += 1  # 计数但不发送
                faults = ["packet_loss"]
                self.stats.faults_injected += 1
            else:
                sentences = self._generate_epoch()
                raw = '\r\n'.join(sentences) + '\r\n'
                data = raw.encode('ascii')
                self._send_to_all(data)
                self.stats.epochs_sent += 1
                self.stats.sentences_sent += len(sentences)
                self.stats.bytes_sent += len(data)

                # 额外延迟 (故障)
                if fc and fc.extra_delay_ms > 0:
                    time.sleep(fc.extra_delay_ms / 1000.0)

            # 控制频率
            elapsed = time.time() - loop_start
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    # ── 连接管理 ──────────────────────────────────────────

    def _accept_loop(self):
        """接受客户端连接"""
        self._server_socket.settimeout(0.5)

        while self._running:
            try:
                client, addr = self._server_socket.accept()
                with self._lock:
                    self._clients.append(client)
                    self.stats.clients_connected += 1
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    self.stats.errors.append(str(e))

    # ── 启动/停止 ─────────────────────────────────────────

    def start(self, host: str = '0.0.0.0', port: int = 5000):
        """启动模拟器 (非阻塞, 后台线程)"""
        if self._running:
            return

        self._server_socket = socket.socket(socket.AF_INET,
                                             socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET,
                                        socket.SO_REUSEADDR, 1)
        self._server_socket.bind((host, port))
        self._server_socket.listen(5)

        self._running = True
        self.stats.start_time = time.time()

        self._thread = threading.Thread(target=self._broadcast_loop,
                                        daemon=True)
        self._thread.start()

        accept_thread = threading.Thread(target=self._accept_loop,
                                         daemon=True)
        accept_thread.start()

        return f"Simulator running on {host}:{port} @ {self.frequency}Hz"

    def stop(self):
        """停止模拟器"""
        self._running = False

        with self._lock:
            for client in self._clients:
                try:
                    client.close()
                except Exception:
                    pass
            self._clients.clear()

        if self._server_socket:
            try:
                self._server_socket.close()
            except Exception:
                pass
            self._server_socket = None

    def is_running(self) -> bool:
        return self._running


# ── 直接运行 ─────────────────────────────────────────────

if __name__ == '__main__':
    LAT, LON, ALT = 22.5431, 113.9408, 50.0

    print("=" * 55)
    print("  GNSS 设备模拟器")
    print("=" * 55)
    print()
    print("  选择场景:")
    print("    0. 静止定位    — RTK固定解, 站在一个点上")
    print("    1. 直线步行    — 1.4 m/s ≈ 5 km/h 行人")
    print("    2. 直线行驶    — 13.9 m/s ≈ 50 km/h 车辆")
    print("    3. 圆周运动    — 绕圈, 适合测试轨迹")
    print("    4. 随机走动    — 模拟行人/GPS漂移")
    print("    5. 信号丢失    — 圆周+故障 (fix跳变/卫星骤降)")
    print("    6. 多星座      — GPS + 北斗 + GLONASS")
    print()
    try:
        choice = input("  输入数字 (0-6, 默认0): ").strip() or "0"
    except (EOFError, KeyboardInterrupt):
        choice = "0"

    # ── 场景配置 ──
    SCENARIOS = {
        '0': ('static', StaticPosition(lat=LAT, lon=LON, altitude=ALT),
              1, ['GP'], None),
        '1': ('linear', LinearMotion(start_lat=LAT, start_lon=LON, altitude=ALT,
                                      speed_mps=1.4, bearing_deg=45),
              1, ['GP'], None),
        '2': ('linear', LinearMotion(start_lat=LAT, start_lon=LON, altitude=ALT,
                                      speed_mps=13.9, bearing_deg=90),
              5, ['GP'], None),
        '3': ('circle', CircleMotion(center_lat=LAT, center_lon=LON,
                                      radius_m=200, speed_mps=5.0, altitude=ALT),
              5, ['GP'], None),
        '4': ('random', RandomWalk(start_lat=LAT, start_lon=LON, altitude=ALT, step_m=2.0),
              1, ['GP'], None),
        '5': ('circle + faults',
              CircleMotion(center_lat=LAT, center_lon=LON, radius_m=200,
                           speed_mps=5.0, altitude=ALT),
              5, ['GP'],
              FaultConfig(fix_degradation_prob=0.5, satellite_drop_prob=0.4,
                          checksum_error_rate=0.3, position_noise_m=5.0)),
        '6': ('multi-constellation',
              StaticPosition(lat=LAT, lon=LON, altitude=ALT),
              1, ['GP', 'BD', 'GL'], None),
    }

    name, traj, freq, talkers, fc = SCENARIOS.get(
        choice, SCENARIOS['0'])

    sim = GnssSimulator(
        trajectory=traj, frequency=freq, talkers=talkers,
        fault_config=fc,
    )

    print()
    print(f"  场景: {name} | 频率: {freq}Hz | 星座: {talkers}")
    print(f"  故障: {'开启' if fc else '关闭'}")
    print("=" * 55)
    print(sim.start(port=5000))
    print()
    print("  新开 Terminal 运行: python receiver.py --output test.nmea")
    print("  按 Ctrl+C 停止...")
    print()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n停止模拟器")
        sim.stop()
        print(f"统计: {sim.stats.epochs_sent} 个历元, "
              f"{sim.stats.sentences_sent} 条语句, "
              f"{sim.stats.faults_injected} 个故障")
