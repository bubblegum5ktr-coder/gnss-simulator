"""
NMEA-0183 语句构造器
====================
将定位数据（经纬度、质量、卫星等）组装成标准 NMEA 语句。
支持 GGA / RMC / GSA / GSV / VTG 五种语句，多星座。
"""
import random
from datetime import datetime


def _nmea_checksum(sentence: str) -> str:
    """计算 NMEA XOR 校验和"""
    ck = 0
    for c in sentence:
        ck ^= ord(c)
    return f"{ck:02X}"


def _dm_lat(deg: float) -> tuple[str, str]:
    """十进制度 → NMEA 纬度 (DDMM.MMMM, N/S)"""
    d = int(abs(deg))
    m = (abs(deg) - d) * 60
    return f"{d:02d}{m:07.4f}", 'N' if deg >= 0 else 'S'


def _dm_lon(deg: float) -> tuple[str, str]:
    """十进制度 → NMEA 经度 (DDDMM.MMMM, E/W)"""
    d = int(abs(deg))
    m = (abs(deg) - d) * 60
    return f"{d:03d}{m:07.4f}", 'E' if deg >= 0 else 'W'


def build_gga(talker: str, utc_time: str, lat: float, lon: float,
              fix_quality: int, satellites: int, hdop: float,
              altitude: float, geoid_sep: float = -999) -> str:
    """
    GGA — 定位信息

    字段: 时间, 纬度, 纬度半球, 经度, 经度半球, 定位质量,
          卫星数, HDOP, 高程, 高程单位, 大地水准面差距, ...
    """
    lat_str, lat_ns = _dm_lat(lat)
    lon_str, lon_ew = _dm_lon(lon)

    # 高程字段: 无效时留空
    if altitude != altitude:  # NaN
        alt_str, alt_unit = '', ''
    else:
        alt_str, alt_unit = f"{altitude:.1f}", 'M'

    # 默认 DGPS 龄期留空
    body = (f"{talker}GGA,{utc_time},{lat_str},{lat_ns},{lon_str},{lon_ew},"
            f"{fix_quality},{satellites:02d},{hdop:.1f},"
            f"{alt_str},{alt_unit},,,,")
    return f"${body}*{_nmea_checksum(body)}"


def build_rmc(talker: str, utc_time: str, status: str,
              lat: float, lon: float, speed_knots: float,
              course: float, date_str: str) -> str:
    """
    RMC — 推荐最小定位信息

    字段: 时间, 状态(A=有效/V=无效), 纬度, 纬度半球, 经度, 经度半球,
          速度(节), 航向, 日期(DDMMYY), 磁偏角, ...
    """
    lat_str, lat_ns = _dm_lat(lat)
    lon_str, lon_ew = _dm_lon(lon)
    body = (f"{talker}RMC,{utc_time},{status},{lat_str},{lat_ns},"
            f"{lon_str},{lon_ew},{speed_knots:.1f},{course:.1f},"
            f"{date_str},,,")
    return f"${body}*{_nmea_checksum(body)}"


def build_gsa(talker: str, mode: str, fix_type: int,
              sat_ids: list[int], pdop: float, hdop: float,
              vdop: float) -> str:
    """
    GSA — 精度因子与活跃卫星

    字段: 模式(M/A), 定位类型(1=无/2=2D/3=3D),
          12个卫星编号槽, PDOP, HDOP, VDOP
    """
    slots = sat_ids[:12]
    slots_str = ','.join(f"{s:02d}" for s in slots)
    slots_str += ',' * (12 - len(slots))  # 补齐 12 个槽位
    body = (f"{talker}GSA,{mode},{fix_type},{slots_str},"
            f"{pdop:.1f},{hdop:.1f},{vdop:.1f}")
    return f"${body}*{_nmea_checksum(body)}"


def build_gsv(talker: str, total_msgs: int, msg_num: int,
              sats_in_view: list[dict], total_sats: int = None) -> str:
    """
    GSV — 可见卫星

    字段: 总条数, 本条序号, 可见卫星总数,
          后面每4个字段一组: 卫星号, 仰角, 方位角, SNR
    每条最多装4颗星, 超过则分多条
    """
    if total_sats is None:
        total_sats = len(sats_in_view)
    body = f"{talker}GSV,{total_msgs},{msg_num},{total_sats:02d}"
    for sat in sats_in_view:
        body += f",{sat['prn']:02d},{sat['elevation']:02d},"
        body += f"{sat['azimuth']:03d},{sat['snr']:02d}"
    return f"${body}*{_nmea_checksum(body)}"


def build_vtg(talker: str, course_true: float, speed_knots: float) -> str:
    """
    VTG — 对地航向与速度

    字段: 真北航向, T(真), 磁北航向, M(磁),
          速度(节), N(节), 速度(km/h), K(km/h)
    """
    speed_kmh = speed_knots * 1.852
    body = (f"{talker}VTG,{course_true:.1f},T,,M,"
            f"{speed_knots:.1f},N,{speed_kmh:.1f},K")
    return f"${body}*{_nmea_checksum(body)}"


def build_epoch(talkers: list[str], utc_time: str, date_str: str,
                lat: float, lon: float, altitude: float,
                fix_quality: int, satellites: int,
                hdop: float, pdop: float, vdop: float,
                speed_knots: float, course: float,
                sat_ids: list[int],
                sats_in_view: list[dict]) -> list[str]:
    """
    生成一个完整历元的全部 NMEA 语句

    返回: NMEA 语句列表，按发送顺序排列
    单星座示例: GGA → GSA → GSV(×n) → RMC → VTG
    多星座示例: GNGGA → GPGSA → BDGSA → GPGSV(×n) → BDGSV(×n) → GNRMC → GNVTG
    """
    sentences = []
    status = 'A' if fix_quality > 0 else 'V'

    # 定位语句: GGA + RMC 用 GN (多系统合并)
    gn_talker = 'GN'
    sentences.append(build_gga(gn_talker, utc_time, lat, lon,
                                fix_quality, satellites, hdop, altitude))
    sentences.append(build_rmc(gn_talker, utc_time, status, lat, lon,
                                speed_knots, course, date_str))
    sentences.append(build_vtg(gn_talker, course, speed_knots))

    # 精度 + 卫星语句: 每个星座各一条 GSA + 各自 GSV
    all_sats_in_view = list(sats_in_view)  # 全部可见星
    for talker in talkers:
        # 给这个星座分配一些卫星 (简单均分)
        # 取属于这个星座的卫星; 为简化, 按 talker 分配前 N 颗
        constellation_sats = [s for s in all_sats_in_view
                              if s.get('talker', 'GP') == talker]
        if not constellation_sats:
            # 自动分配: 每个星座分一部分
            n = max(1, len(all_sats_in_view) // len(talkers))
            offset = talkers.index(talker) * n
            constellation_sats = all_sats_in_view[offset:offset + n]

        # GSA: 用这个星座的活跃卫星
        gsa_ids = sat_ids if talker == 'GP' else [s['prn'] for s in constellation_sats]
        sentences.append(build_gsa(talker, 'A', 3, gsa_ids, pdop, hdop, vdop))

        # GSV: 分组, 每组最多4颗
        n_total = len(constellation_sats)
        for i in range(0, n_total, 4):
            group = constellation_sats[i:i+4]
            total = (n_total + 3) // 4
            msg_num = i // 4 + 1
            sentences.append(build_gsv(talker, total, msg_num, group,
                                       total_sats=n_total))

    return sentences
