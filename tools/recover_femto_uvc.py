#!/usr/bin/env python3
"""诊断 Femto Bolt UVC 节点；--apply 只重绑定缺失的相机视频控制接口。"""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess

USB = Path('/sys/bus/usb/devices')
DRIVER = Path('/sys/bus/usb/drivers/uvcvideo')


def read(path):
    return path.read_text().strip()


def video_nodes(interface):
    return sorted('/dev/' + p.name for p in Path('/sys/class/video4linux').glob('*')
                  if (p / 'device').resolve() == interface.resolve())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--usb-port', help='例如 2-1.3；多台相机时必须指定')
    parser.add_argument('--apply', action='store_true', help='在无占用时恢复缺失接口，需 sudo')
    args = parser.parse_args()
    if args.usb_port and not re.fullmatch(r'[0-9]+-[0-9]+(?:\.[0-9]+)*', args.usb_port):
        parser.error('Invalid USB port name')
    cameras = []
    for p in USB.iterdir():
        if not (p / 'idVendor').exists() or not (p / 'idProduct').exists():
            continue
        if read(p / 'idVendor').lower() == '2bc5' and read(p / 'idProduct').lower() == '066b':
            if not args.usb_port or p.name == args.usb_port:
                cameras.append(p)
    if len(cameras) != 1:
        parser.error('Expected exactly one Femto Bolt; use --usb-port to select it')
    camera = cameras[0]
    interfaces = [p for p in USB.glob(camera.name + ':*')
                  if (p / 'bInterfaceClass').exists()
                  and read(p / 'bInterfaceClass') == '0e'
                  and read(p / 'bInterfaceSubClass') == '01']
    if len(interfaces) != 2:
        raise RuntimeError('Unexpected Femto Bolt USB interfaces; refusing to modify them')
    missing = sorted((p for p in interfaces if not video_nodes(p)), key=lambda p: p.name)
    result = {'usb_device': camera.name,
              'missing_before': [p.name for p in missing], 'rebound': []}
    if missing and args.apply:
        if os.geteuid() != 0:
            parser.error('Use sudo with --apply after stopping all camera programs')
        if not DRIVER.is_dir():
            raise RuntimeError('uvcvideo is not loaded; no changes made')
        device = '/dev/bus/usb/{:03d}/{:03d}'.format(
            int(read(camera / 'busnum')), int(read(camera / 'devnum')))
        for interface in missing:
            # 每次修改前再次确认没有 SDK、Viewer 或其他程序占用此 USB 设备。
            targets = [device, *(node for p in interfaces for node in video_nodes(p))]
            busy = subprocess.run(['fuser', *targets], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            if busy.returncode != 1:
                raise RuntimeError('Camera USB device is busy or occupancy check failed')
            current = (interface / 'driver').resolve() if (interface / 'driver').exists() else None
            if current is not None and current != DRIVER:
                raise RuntimeError('Unexpected USB driver; refusing to rebind')
            if current == DRIVER:
                (DRIVER / 'unbind').write_text(interface.name)
            (DRIVER / 'bind').write_text(interface.name)
            result['rebound'].append(interface.name)
        subprocess.run(['udevadm', 'settle', '--timeout=5'], check=True)
    result['interfaces'] = {p.name: video_nodes(p) for p in sorted(interfaces)}
    result['healthy'] = all(result['interfaces'].values())
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['healthy'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
