#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Download explore map from the car; optionally delete remote copies."""
from __future__ import print_function

import argparse
import os
import struct
import sys
from datetime import datetime

try:
    import configparser
    import paramiko
except ImportError:
    print('Need paramiko: pip install paramiko', file=sys.stderr)
    sys.exit(1)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
INI_PATH = os.path.join(SCRIPT_DIR, 'upload_to_car.ini')
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..'))
REMOTE_DIR = '/home/hiwonder/ros_ws/src/hiwonder_slam/maps'
NAMES = ('explore.pgm', 'explore.yaml')


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--name', default='', help='backup folder name under maps_backup/')
    p.add_argument('--keep-remote', action='store_true', help='do not delete files on the car')
    return p.parse_args()


def read_pgm(path):
    with open(path, 'rb') as f:
        magic = f.readline().strip()
        if magic != b'P5':
            raise ValueError('not a binary PGM: %s' % path)
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        width, height = [int(x) for x in line.split()]
        maxval = int(f.readline().strip())
        data = f.read()
    if maxval != 255:
        raise ValueError('expected maxval 255')
    if len(data) != width * height:
        raise ValueError('pgm size mismatch')
    return width, height, data


def write_bmp8(path, width, height, data):
    """8-bit grayscale BMP, origin top-left to match PGM/RViz."""
    row_pad = (4 - (width % 4)) % 4
    palette = b''.join(struct.pack('<BBBB', i, i, i, 0) for i in range(256))
    pixel_size = (width + row_pad) * height
    off_bits = 54 + 1024
    file_size = off_bits + pixel_size
    header = struct.pack(
        '<2sIHHIIiiHHIIiiII',
        b'BM', file_size, 0, 0, off_bits,
        40, width, -height, 1, 8, 0, pixel_size, 2835, 2835, 256, 256
    )
    rows = []
    for y in range(height):
        rows.append(data[y * width:(y + 1) * width] + (b'\x00' * row_pad))
    with open(path, 'wb') as f:
        f.write(header)
        f.write(palette)
        f.write(b''.join(rows))


def write_note(folder):
    text = (
        '本目录是车上第二次建图备份（下午），与上午那份分开。\n'
        '\n'
        '  maps_backup/2026-09-18-explore/     上午第一张（已从车上删过）\n'
        '  maps_backup/2026-09-18-explore-v2/  本次新建，车上 explore 仍保留\n'
        '\n'
        '颜色（ROS 栅格图）：\n'
        '  黑(0)   = 障碍，台子/围栏请涂黑\n'
        '  白(254) = 可走\n'
        '  灰(205) = 未知，场外扇形请涂灰，不要留白\n'
        '\n'
        '建议改 explore_for_paint.bmp（画图能开），改完另存为 24 位 BMP 也可。\n'
        '导航实际用的是 explore.pgm + explore.yaml，yaml 不要改分辨率和原点。\n'
        '改完需要我再转回 pgm 并传上车再说一声。\n'
    )
    with open(os.path.join(folder, '说明.txt'), 'w', encoding='utf-8') as f:
        f.write(text)


def load_car():
    if not os.path.isfile(INI_PATH):
        raise FileNotFoundError('missing %s' % INI_PATH)
    cfg = configparser.ConfigParser()
    cfg.read(INI_PATH, encoding='utf-8')
    host = cfg.get('car', 'host').strip()
    user = cfg.get('car', 'user').strip()
    password = cfg.get('car', 'password').strip()
    port = 22
    if cfg.has_option('car', 'port'):
        port = int(cfg.get('car', 'port').strip() or '22')
    timeout = 15.0
    if cfg.has_option('car', 'connect_timeout'):
        timeout = float(cfg.get('car', 'connect_timeout').strip() or '15')
    if not host or not user or not password or ('填密码' in password):
        raise ValueError('ini host/user/password not ready')
    return host, port, user, password, timeout


def main():
    args = parse_args()
    folder_name = args.name.strip() or (datetime.now().strftime('%Y-%m-%d') + '-explore')
    local_dir = os.path.join(REPO_ROOT, 'maps_backup', folder_name)
    host, port, user, password, timeout = load_car()
    os.makedirs(local_dir, exist_ok=True)
    print('host=%s user=%s' % (host, user))
    print('local_dir=%s' % local_dir)

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(
        hostname=host,
        port=port,
        username=user,
        password=password,
        timeout=timeout,
        allow_agent=False,
        look_for_keys=False,
    )
    sftp = client.open_sftp()
    print('SSH connected')

    pulled = []
    for name in NAMES:
        remote = '%s/%s' % (REMOTE_DIR, name)
        local = os.path.join(local_dir, name)
        try:
            st = sftp.stat(remote)
        except IOError:
            print('MISSING on car: %s' % remote)
            sftp.close()
            client.close()
            return 1
        print('download %s (%d bytes)' % (remote, st.st_size))
        sftp.get(remote, local)
        loc_size = os.path.getsize(local)
        if loc_size <= 0 or loc_size != st.st_size:
            print('VERIFY FAIL %s remote=%d local=%d' % (name, st.st_size, loc_size))
            sftp.close()
            client.close()
            return 1
        pulled.append((name, loc_size))

    if args.keep_remote:
        print('keep remote, skip delete')
    else:
        print('local copy ok, deleting remote...')
        for name in NAMES:
            remote = '%s/%s' % (REMOTE_DIR, name)
            sftp.remove(remote)
            try:
                sftp.stat(remote)
                print('DELETE FAIL still exists: %s' % remote)
                sftp.close()
                client.close()
                return 1
            except IOError:
                print('deleted %s' % remote)

    sftp.close()
    client.close()

    pgm = os.path.join(local_dir, 'explore.pgm')
    bmp = os.path.join(local_dir, 'explore_for_paint.bmp')
    try:
        w, h, data = read_pgm(pgm)
        write_bmp8(bmp, w, h, data)
        print('paint bmp %s (%dx%d)' % (bmp, w, h))
    except Exception as e:
        print('bmp convert skip: %s' % e)
    write_note(local_dir)

    print('DONE')
    for name, size in pulled:
        print('saved %s (%d bytes)' % (os.path.join(local_dir, name), size))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as e:
        print('ERROR: %s' % e, file=sys.stderr)
        sys.exit(1)
