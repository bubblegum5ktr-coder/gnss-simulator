"""
轨迹生成器
=========
提供多种运动轨迹的坐标序列，供模拟器驱动位置变化。

每条轨迹从初始位置开始，按时间推进产生新的定位坐标。
"""
import math
import random
import time
from dataclasses import dataclass


@dataclass
class Position:
    """一个历元的定位数据"""
    lat: float = 22.5          # 纬度 (十进制度)
    lon: float = 113.0         # 经度 (十进制度)
    altitude: float = 50.0     # 高程 (米)
    fix_quality: int = 4       # 定位质量 (4=RTK固定)
    satellites: int = 18       # 参与定位卫星数
    hdop: float = 0.6          # 水平精度因子
    pdop: float = 1.2          # 位置精度因子
    vdop: float = 1.5          # 垂直精度因子
    speed_knots: float = 0.0   # 速度 (节)
    course: float = 0.0        # 航向 (度真北)
    sat_ids: tuple = ()        # 活跃卫星编号
    sats_in_view: tuple = ()   # 可见卫星详情 [{prn, elevation, azimuth, snr}, ...]


class StaticPosition:
    """静止不动 —— 适合测试基本输出"""

    def __init__(self, lat=22.5431, lon=113.9408, altitude=50.0):
        self.lat = lat
        self.lon = lon
        self.altitude = altitude
        self._prev = None

    def next(self) -> Position:
        return Position(
            lat=self.lat, lon=self.lon, altitude=self.altitude,
            speed_knots=0.0, course=0.0
        )


class LinearMotion:
    """匀速直线运动 —— 模拟车辆/人员沿直线行进"""

    def __init__(self, start_lat=22.5431, start_lon=113.9408,
                 altitude=50.0, speed_mps=1.4, bearing_deg=45.0):
        """
        speed_mps: 速度 (米/秒), 默认 1.4 m/s ≈ 5 km/h (步行)
        bearing_deg: 方位角 (度真北, 0=北 90=东)
        """
        self.lat = start_lat
        self.lon = start_lon
        self.altitude = altitude
        self.speed_mps = speed_mps
        self.bearing_rad = math.radians(bearing_deg)
        self._last_time = None

    def next(self) -> Position:
        now = time.time()
        if self._last_time is None:
            self._last_time = now
            return Position(lat=self.lat, lon=self.lon,
                            altitude=self.altitude, speed_knots=0, course=0)

        dt = now - self._last_time
        self._last_time = now
        distance_m = self.speed_mps * dt
        speed_knots = self.speed_mps * 1.94384

        # 球面近似: 纬度方向 1° ≈ 111320 m, 经度方向需×cos(lat)
        dlat = (distance_m * math.cos(self.bearing_rad)) / 111320.0
        dlon = (distance_m * math.sin(self.bearing_rad)) / (
            111320.0 * math.cos(math.radians(self.lat)))

        self.lat += dlat
        self.lon += dlon
        course_deg = math.degrees(self.bearing_rad)

        return Position(
            lat=self.lat, lon=self.lon, altitude=self.altitude,
            speed_knots=speed_knots, course=course_deg
        )


class CircleMotion:
    """圆周运动 —— 模拟车辆绕圈/测试轨迹"""

    def __init__(self, center_lat=22.5431, center_lon=113.9408,
                 radius_m=100.0, speed_mps=1.4, altitude=50.0):
        """
        radius_m: 半径 (米)
        speed_mps: 线速度 (米/秒)
        """
        self.center_lat = center_lat
        self.center_lon = center_lon
        self.radius_m = radius_m
        self.speed_mps = speed_mps
        self.altitude = altitude
        self.angle = 0.0  # 当前角度 (弧度)
        self._last_time = None

    def next(self) -> Position:
        now = time.time()
        if self._last_time is None:
            self._last_time = now
            # 起点: 圆心正北方向 (半径距离)
            dlat = self.radius_m / 111320.0
            return Position(
                lat=self.center_lat + dlat, lon=self.center_lon,
                altitude=self.altitude, speed_knots=0, course=0
            )

        dt = now - self._last_time
        self._last_time = now

        # 角速度 = 线速度 / 半径
        angular_vel = self.speed_mps / self.radius_m
        self.angle += angular_vel * dt
        speed_knots = self.speed_mps * 1.94384

        # 圆周上的位置
        dlat = (self.radius_m * math.cos(self.angle)) / 111320.0
        dlon = (self.radius_m * math.sin(self.angle)) / (
            111320.0 * math.cos(math.radians(self.center_lat)))

        # 航向 = 切线方向 (角度 + 90°)
        course_deg = math.degrees(self.angle + math.pi / 2) % 360

        return Position(
            lat=self.center_lat + dlat, lon=self.center_lon + dlon,
            altitude=self.altitude, speed_knots=speed_knots,
            course=course_deg
        )


class RandomWalk:
    """随机走动 —— 模拟行人/GPS 漂移"""

    def __init__(self, start_lat=22.5431, start_lon=113.9408,
                 altitude=50.0, step_m=2.0, bounds=None):
        """
        step_m: 每秒最大步长 (米)
        bounds: (min_lat, max_lat, min_lon, max_lon) 活动范围
        """
        self.lat = start_lat
        self.lon = start_lon
        self.altitude = altitude
        self.step_m = step_m
        self.bounds = bounds
        self._last_time = None

    def next(self) -> Position:
        now = time.time()
        if self._last_time is None:
            self._last_time = now
            return Position(lat=self.lat, lon=self.lon,
                            altitude=self.altitude, speed_knots=0, course=0)

        dt = now - self._last_time
        self._last_time = now

        # 随机方向和步长
        angle = random.uniform(0, 2 * math.pi)
        distance = random.uniform(0, self.step_m * dt)
        speed_knots = (distance / dt) * 1.94384 if dt > 0 else 0

        dlat = (distance * math.cos(angle)) / 111320.0
        dlon = (distance * math.sin(angle)) / (
            111320.0 * math.cos(math.radians(self.lat)))

        self.lat += dlat
        self.lon += dlon

        # 边界约束
        if self.bounds:
            self.lat = max(self.bounds[0], min(self.bounds[1], self.lat))
            self.lon = max(self.bounds[2], min(self.bounds[3], self.lon))

        course_deg = math.degrees(angle) % 360

        return Position(
            lat=self.lat, lon=self.lon, altitude=self.altitude,
            speed_knots=speed_knots, course=course_deg
        )
