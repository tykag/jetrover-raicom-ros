#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Convert edited BMP to ROS PGM and upload explore map to the car."""
from __future__ import print_function

import configparser
import os
import struct
import sys
from datetime import datetime

import paramiko

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
INI_PATH = os.path.join(SCRIPT_DIR, 'upload_to_car.ini')
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..'))
LOCAL_DIR = os.path.join(REPO_ROOT, 'maps_backup', '2026-09-18-explore-v2')
BMP_PATH = os.path.join(LOCAL_DIR, 'explore_for_paint.bmp')
OUT_PGM = os.path.join(LOCAL_DIR, 'explore_edited.pgm')
YAML_PATH = os.path.join(LOCAL_DIR, 'explore.yaml')
REMOTE_DIR = '/home/hiwonder/ros_ws/src/hiwonder_slam/maps'


def load_car():
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
    return host, port, user, password, timeout


def classify(v):
    if v <= 80:
        return 0
    if v >= 240:
        return 254
    return 205


def read_bmp_gray(path):
    data = open(path, 'rb').read()
    if data[:2] != b'BM':
        raise ValueError('not a BMP')
    off = struct.unpack_from('<I', data, 10)[0]
    header_size = struct.unpack_from('<I', data, 14)[0]
    w, h = struct.unpack_from('<ii', data, 18)
    bits = struct.unpack_from('<H', data, 28)[0]
    top_down = h < 0
    h = abs(h)
    row_bytes = ((w * bits + 31) // 32) * 4
    raw = data[off:]
    pixels = []
    for y in range(h):
        src = y * row_bytes
        row = []
        if bits == 8:
            row = [classify(raw[src + x]) for x in range(w)]
        elif bits == 24:
            for x in range(w):
                b = raw[src + x * 3]
                g = raw[src + x * 3 + 1]
                r = raw[src + x * 3 + 2]
                row.append(classify((int(r) + int(g) + int(b)) // 3))
        elif bits == 32:
            for x in range(w):
                b = raw[src + x * 4]
                g = raw[src + x * 4 + 1]
                r = raw[src + x * 4 + 2]
                row.append(classify((int(r) + int(g) + int(b)) // 3))
        else:
            raise ValueError('unsupported BMP bits=%s' % bits)
        pixels.append(row)
    if not top_down:
        pixels.reverse()
    print('bmp %dx%d bits=%d top_down=%s header=%d' % (w, h, bits, top_down, header_size))
    return w, h, pixels


def write_pgm(path, w, h, pixels):
    body = bytearray()
    counts = {0: 0, 205: 0, 254: 0}
    for y in range(h):
        for x in range(w):
            v = pixels[y][x]
            body.append(v)
            counts[v] = counts.get(v, 0) + 1
    with open(path, 'wb') as f:
        f.write(('P5\n%d %d\n255\n' % (w, h)).encode('ascii'))
        f.write(bytes(body))
    print('pgm %s counts=%s' % (path, counts))


def main():
    w, h, pixels = read_bmp_gray(BMP_PATH)
    if (w, h) != (416, 448):
        print('WARN size %dx%d (original was 416x448)' % (w, h))
    write_pgm(OUT_PGM, w, h, pixels)
    if not os.path.isfile(YAML_PATH):
        raise FileNotFoundError(YAML_PATH)

    host, port, user, password, timeout = load_car()
    print('upload to %s@%s' % (user, host))
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(
        hostname=host, port=port, username=user, password=password,
        timeout=timeout, allow_agent=False, look_for_keys=False,
    )
    sftp = client.open_sftp()
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    remote_pgm = REMOTE_DIR + '/explore.pgm'
    remote_yaml = REMOTE_DIR + '/explore.yaml'
    try:
        sftp.stat(remote_pgm)
        bak = REMOTE_DIR + '/explore_before_edit_%s.pgm' % stamp
        sftp.rename(remote_pgm, bak)
        print('backed up car pgm -> %s' % bak)
    except IOError:
        print('no existing explore.pgm on car')

    sftp.put(OUT_PGM, remote_pgm)
    sftp.put(YAML_PATH, remote_yaml)
    st = sftp.stat(remote_pgm)
    print('uploaded pgm %d bytes yaml %d bytes' % (st.st_size, sftp.stat(remote_yaml).st_size))
    sftp.close()
    client.close()
    print('DONE')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as e:
        print('ERROR: %s' % e, file=sys.stderr)
        sys.exit(1)
