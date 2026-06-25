"""
GNSS 设备模拟器 — 测试套件
=========================
"""
import pytest
import sys
import time

sys.path.insert(0, 'D:/workspace/gnss-simulator')

from nmea_builder import (build_gga, build_rmc, build_gsa, build_gsv,
                          build_vtg, build_epoch)
from trajectory import StaticPosition, LinearMotion, CircleMotion, RandomWalk, Position
from simulator import GnssSimulator, FaultConfig


# ════════════════════════════════════════════════════════
#  NMEA Builder 测试
# ════════════════════════════════════════════════════════

class TestNmeaBuilder:
    def test_gga_format(self):
        gga = build_gga('GN', '083000.00', 22.5431, 113.9408, 4, 18, 0.6, 50.0)
        assert gga.startswith('$GNGGA')
        assert '*17' in gga  # known checksum
        assert '22' in gga and '113' in gga

    def test_rmc_format(self):
        rmc = build_rmc('GN', '083000.00', 'A', 22.0, 113.0, 5.2, 120.0, '240624')
        assert rmc.startswith('$GNRMC')
        assert 'A' in rmc and '240624' in rmc

    def test_checksum_is_valid(self):
        for talker in ['GP', 'BD', 'GN', 'GL']:
            gga = build_gga(talker, '083000.00', 22.5, 113.0, 4, 18, 0.6, 50.0)
            body, cksum = gga[1:].split('*')
            expected = 0
            for c in body:
                expected ^= ord(c)
            assert cksum == f"{expected:02X}"

    def test_gsa_fills_12_slots(self):
        gsa = build_gsa('GP', 'A', 3, [1, 3, 4], 1.2, 0.6, 1.5)
        # 12 satellite slots after fix_type
        parts = gsa.split(',')
        assert len(parts) == 18  # header + mode + type + 12 slots + PDOP + HDOP + VDOP*checksum

    def test_gsv_groups_satellites(self):
        sats = [{'prn': i, 'elevation': 45, 'azimuth': 90, 'snr': 42}
                for i in range(1, 9)]  # 8 sats → 2 messages
        gsv1 = build_gsv('GP', 2, 1, sats[:4], total_sats=8)
        gsv2 = build_gsv('GP', 2, 2, sats[4:], total_sats=8)
        assert '$GPGSV,2,1,08' in gsv1
        assert '$GPGSV,2,2,08' in gsv2

    def test_vtg_format(self):
        vtg = build_vtg('GN', 120.0, 10.0)
        assert '$GNVTG' in vtg
        assert '120.0,T' in vtg

    def test_build_epoch_single_constellation(self):
        sats = [{'prn': 1, 'talker': 'GP', 'elevation': 45,
                 'azimuth': 90, 'snr': 42}]
        epoch = build_epoch(
            talkers=['GP'], utc_time='083000', date_str='240624',
            lat=22.5, lon=113.0, altitude=50.0,
            fix_quality=4, satellites=18, hdop=0.6, pdop=1.2, vdop=1.5,
            speed_knots=0.0, course=0.0,
            sat_ids=[1], sats_in_view=sats
        )
        types = [s[1:5] for s in epoch]
        assert 'GNGG' in types  # GGA
        assert 'GNRM' in types  # RMC
        assert 'GPGS' in types  # GPGSA + GPGSV

    def test_build_epoch_multi_constellation(self):
        sats = [
            {'prn': 1, 'talker': 'GP', 'elevation': 45, 'azimuth': 90, 'snr': 42},
            {'prn': 30, 'talker': 'BD', 'elevation': 30, 'azimuth': 270, 'snr': 35},
        ]
        epoch = build_epoch(
            talkers=['GP', 'BD'], utc_time='083000', date_str='240624',
            lat=22.5, lon=113.0, altitude=50.0,
            fix_quality=4, satellites=18, hdop=0.6, pdop=1.2, vdop=1.5,
            speed_knots=0.0, course=0.0,
            sat_ids=[1, 30], sats_in_view=sats
        )
        # Should have both GP and BD sentences
        full = ' '.join(epoch)
        assert 'GP' in full
        assert 'BD' in full

    def test_lat_lon_conversion(self):
        # 22.5431° → 22°32.586' N → "2232.5860,N"
        gga = build_gga('GN', '083000.00', 22.5431, 113.9408, 4, 18, 0.6, 50.0)
        assert '2232.5860,N' in gga
        assert '11356.4480,E' in gga

    def test_southern_hemisphere(self):
        gga = build_gga('GN', '083000.00', -33.8688, 151.2093, 4, 18, 0.6, 50.0)
        assert ',S' in gga
        assert '15112.5580,E' in gga

    def test_invalid_fix_shows_v_status(self):
        sats = [{'prn': 1, 'talker': 'GP', 'elevation': 45,
                 'azimuth': 90, 'snr': 42}]
        epoch = build_epoch(
            talkers=['GP'], utc_time='083000', date_str='240624',
            lat=22.5, lon=113.0, altitude=50.0,
            fix_quality=0, satellites=0, hdop=99.9, pdop=99.9, vdop=99.9,
            speed_knots=0.0, course=0.0,
            sat_ids=[], sats_in_view=sats
        )
        # RMC should show V (invalid)
        rmc = [s for s in epoch if 'RMC' in s][0]
        assert ',V,' in rmc


# ════════════════════════════════════════════════════════
#  轨迹测试
# ════════════════════════════════════════════════════════

class TestTrajectory:
    def test_static_returns_same_position(self):
        traj = StaticPosition(lat=22.5, lon=113.0)
        p1 = traj.next()
        p2 = traj.next()
        assert p1.lat == p2.lat == 22.5
        assert p1.lon == p2.lon == 113.0
        assert p1.speed_knots == 0.0

    def test_linear_motion_moves(self):
        traj = LinearMotion(start_lat=22.5, start_lon=113.0,
                            speed_mps=1.4, bearing_deg=90)  # 正东
        p1 = traj.next()
        time.sleep(1.1)
        p2 = traj.next()
        # 1秒后应向东移动
        assert p2.lon > p1.lon

    def test_circle_motion_starts_north_of_center(self):
        traj = CircleMotion(center_lat=22.5, center_lon=113.0,
                            radius_m=100.0)
        p = traj.next()
        assert p.lat > 22.5  # 起点在圆心北边

    def test_random_walk_moves(self):
        traj = RandomWalk(start_lat=22.5, start_lon=113.0, step_m=2.0)
        p1 = traj.next()
        time.sleep(0.5)
        p2 = traj.next()
        # 0.5秒后应有轻微移动
        assert p1.lat != p2.lat or p1.lon != p2.lon


# ════════════════════════════════════════════════════════
#  模拟器测试
# ════════════════════════════════════════════════════════

class TestSimulator:
    def test_start_and_stop(self):
        sim = GnssSimulator(frequency=5)
        msg = sim.start(port=5006)
        assert sim.is_running()
        assert '5006' in msg
        sim.stop()
        assert not sim.is_running()

    def test_default_trajectory_is_static(self):
        sim = GnssSimulator()
        assert sim.trajectory is not None

    def test_stats_recorded(self):
        sim = GnssSimulator(frequency=10)
        sim.start(port=5007)
        time.sleep(0.5)
        sim.stop()
        assert sim.stats.epochs_sent > 0
        assert sim.stats.sentences_sent > 0

    def test_fault_config_defaults(self):
        sim = GnssSimulator()
        assert sim.fault_config.packet_loss_rate == 0.0
        assert sim.fault_config.fix_degradation_prob == 0.0

    def test_fault_injection_stats(self):
        fc = FaultConfig(
            fix_degradation_prob=1.0,   # 100% → 一定触发
            position_noise_m=5.0,
        )
        sim = GnssSimulator(
            trajectory=StaticPosition(),
            frequency=10,
            fault_config=fc,
        )
        sim.start(port=5008)
        time.sleep(0.5)
        sim.stop()
        assert sim.stats.faults_injected > 0

    def test_multiple_stop_is_safe(self):
        sim = GnssSimulator()
        sim.start(port=5009)
        sim.stop()
        sim.stop()  # 不应崩溃

    def test_multi_constellation(self):
        sim = GnssSimulator(talkers=['GP', 'BD', 'GL'], frequency=5)
        assert len(sim.talkers) == 3
        sim.start(port=5010)
        time.sleep(0.3)
        sim.stop()
