"""
NMEA 数据接收 & 校验
===================
连接 GNSS 模拟器，接收 NMEA 数据，可选保存为文件或直接用 validator 校验。

用法:
    # 只接收并打印
    python receiver.py

    # 保存到文件
    python receiver.py --output captured.nmea --duration 10

    # 接收 + 自动校验(需要项目1 nmea-validator)
    python receiver.py --validate
"""
import socket
import sys
import os
import time


def receive(host='localhost', port=5000, duration=5):
    """从 TCP 接收 NMEA 数据, 返回字符串"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)
    try:
        sock.connect((host, port))
    except Exception:
        return ""

    all_data = b''
    deadline = time.time() + duration
    sock.settimeout(0.2)

    while time.time() < deadline:
        try:
            chunk = sock.recv(65536)
            if chunk:
                all_data += chunk
        except socket.timeout:
            pass
        except Exception:
            break

    sock.close()
    return all_data.decode('ascii', errors='ignore')


def save_to_file(data: str, filepath: str):
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write(data)
    print(f"已保存到: {filepath}")


def validate_data(data: str, validator_path: str = None):
    """用 nmea-validator 校验数据

    validator_path: nmea-validator 项目路径, 默认从以下来源推断:
        1. 命令行 --validator-path 参数
        2. 环境变量 NMEA_VALIDATOR_PATH
        3. 默认路径 D:/workspace/nmea-validator
    """
    import os as _os
    if validator_path is None:
        validator_path = _os.environ.get(
            'NMEA_VALIDATOR_PATH',
            'D:/workspace/nmea-validator'
        )
    sys.path.insert(0, validator_path)
    import tempfile
    from nmea_validator import validate_file

    with tempfile.NamedTemporaryFile(mode='w', suffix='.nmea', delete=False) as f:
        f.write(data)
        tmpfile = f.name

    result = validate_file(tmpfile)
    print(result.summary())
    _os.unlink(tmpfile)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='接收 GNSS 模拟器 NMEA 数据')
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=5000)
    parser.add_argument('--duration', type=int, default=5,
                        help='接收时长(秒)')
    parser.add_argument('--output', '-o', help='保存到文件')
    parser.add_argument('--validate', '-v', action='store_true',
                        help='接收后用 validator 校验')
    parser.add_argument('--validator-path',
                        default=None,
                        help='nmea-validator 项目路径 (默认: $NMEA_VALIDATOR_PATH 或 D:/workspace/nmea-validator)')
    args = parser.parse_args()

    print(f"连接 {args.host}:{args.port} ...")
    data = receive(args.host, args.port, args.duration)

    line_count = data.strip().count('\n') + 1
    print(f"收到 {line_count} 条语句, {len(data)} 字节")

    if not data.strip():
        print("未收到数据! 确认模拟器正在运行:")
        print("  python simulator.py")
        sys.exit(1)

    if args.output:
        save_to_file(data, args.output)

    if args.validate:
        validate_data(data, validator_path=args.validator_path)
    elif not args.output:
        print(data[:2000])
        if len(data) > 2000:
            print(f"\n... (共 {len(data)} 字节, 用 --output 保存完整数据)")
