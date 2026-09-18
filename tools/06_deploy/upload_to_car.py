#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把本地文件上传到小车，失败则自动回滚备份。

用法：
  python upload_to_car.py
  python upload_to_car.py --config upload_to_car.ini
  python upload_to_car.py --no-restart

依赖：pip install paramiko
配置：同目录 upload_to_car.ini（可从 .example 复制）
日志：logs/upload_YYYYMMDD_HHMMSS.txt
"""
from __future__ import print_function

import argparse
import configparser
import hashlib
import os
import sys
import time
import traceback
from datetime import datetime

try:
    import paramiko
except ImportError:
    print('缺少 paramiko，请先执行: pip install paramiko', file=sys.stderr)
    sys.exit(1)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(SCRIPT_DIR, 'upload_to_car.ini')
LOG_DIR = os.path.join(SCRIPT_DIR, 'logs')


class TeeLogger(object):
    def __init__(self, log_path):
        self.log_path = log_path
        self._fh = open(log_path, 'a', encoding='utf-8')

    def close(self):
        try:
            self._fh.close()
        except Exception:
            pass

    def log(self, msg, also_print=True):
        line = '[%s] %s' % (datetime.now().strftime('%Y-%m-%d %H:%M:%S'), msg)
        self._fh.write(line + '\n')
        self._fh.flush()
        if also_print:
            print(line)


def md5_file(path, chunk=1024 * 1024):
    h = hashlib.md5()
    with open(path, 'rb') as f:
        while True:
            buf = f.read(chunk)
            if not buf:
                break
            h.update(buf)
    return h.hexdigest()


def md5_remote(sftp, path, chunk=1024 * 1024):
    h = hashlib.md5()
    with sftp.open(path, 'rb') as f:
        while True:
            buf = f.read(chunk)
            if not buf:
                break
            h.update(buf)
    return h.hexdigest()


def load_config(path):
    if not os.path.isfile(path):
        raise FileNotFoundError(
            '找不到配置文件: %s\n请先复制 upload_to_car.ini.example 为 upload_to_car.ini 并填写 IP/账号密码'
            % path
        )
    cfg = configparser.ConfigParser()
    # 保留键名大小写风格，统一小写读
    cfg.optionxform = str
    read_ok = cfg.read(path, encoding='utf-8')
    if not read_ok:
        raise RuntimeError('无法读取配置: %s' % path)

    def get(section, key, fallback=None):
        if cfg.has_option(section, key):
            return cfg.get(section, key).strip()
        if fallback is not None:
            return fallback
        raise KeyError('[%s] 缺少 %s' % (section, key))

    def getbool(section, key, fallback=False):
        if not cfg.has_option(section, key):
            return fallback
        return cfg.getboolean(section, key)

    car = {
        'host': get('car', 'host'),
        'port': int(get('car', 'port', '22')),
        'user': get('car', 'user'),
        'password': get('car', 'password'),
        'connect_timeout': float(get('car', 'connect_timeout', '15')),
    }
    if not car['host'] or not car['user']:
        raise ValueError('ini 里 [car] host / user 不能为空')
    if not car['password']:
        raise ValueError('ini 里 [car] password 不能为空（不要写死在 .py 里）')

    files = {
        'local': get('files', 'local'),
        'remote': get('files', 'remote'),
    }
    # 相对路径相对 ini 所在目录的上两级（源码资料根）或脚本目录
    root = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..'))
    local_path = files['local']
    if not os.path.isabs(local_path):
        cand = os.path.join(root, local_path)
        if os.path.isfile(cand):
            local_path = cand
        else:
            local_path = os.path.join(SCRIPT_DIR, files['local'])
    files['local'] = os.path.abspath(local_path)

    options = {
        'backup': getbool('options', 'backup', True),
        'verify_md5': getbool('options', 'verify_md5', True),
        'restart_joystick': getbool('options', 'restart_joystick', True),
        'strip_crlf': getbool('options', 'strip_crlf', True),
        'remote_shell': get('options', 'remote_shell', 'bash'),
    }
    return car, files, options


def ssh_connect(car, logger):
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    logger.log('连接 %s@%s:%s ...' % (car['user'], car['host'], car['port']))
    client.connect(
        hostname=car['host'],
        port=car['port'],
        username=car['user'],
        password=car['password'],
        timeout=car['connect_timeout'],
        allow_agent=False,
        look_for_keys=False,
    )
    logger.log('SSH 已连接')
    return client


def run_remote(client, cmd, logger, check=True):
    logger.log('远程执行: %s' % cmd)
    stdin, stdout, stderr = client.exec_command(cmd, get_pty=True)
    code = stdout.channel.recv_exit_status()
    out = stdout.read().decode('utf-8', errors='replace').strip()
    err = stderr.read().decode('utf-8', errors='replace').strip()
    if out:
        logger.log('stdout: %s' % out)
    if err:
        logger.log('stderr: %s' % err)
    if check and code != 0:
        raise RuntimeError('远程命令失败 (exit=%s): %s' % (code, cmd))
    return code, out, err


def remote_exists(sftp, path):
    try:
        sftp.stat(path)
        return True
    except IOError:
        return False


def upload_with_rollback(car, files, options, logger, do_restart=True):
    local = files['local']
    remote = files['remote']
    if not os.path.isfile(local):
        raise FileNotFoundError('本地文件不存在: %s' % local)

    local_size = os.path.getsize(local)
    local_md5 = md5_file(local) if options['verify_md5'] else None
    logger.log('本地: %s (%d bytes%s)' % (
        local, local_size, (', md5=' + local_md5) if local_md5 else ''))
    logger.log('远程: %s' % remote)

    client = None
    sftp = None
    backup_path = None
    uploaded = False
    had_original = False

    try:
        client = ssh_connect(car, logger)
        sftp = client.open_sftp()

        # 1) 备份
        if options['backup'] and remote_exists(sftp, remote):
            had_original = True
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            backup_path = remote + '.bak.' + stamp
            logger.log('备份远程文件 -> %s' % backup_path)
            run_remote(client, 'cp -a %s %s' % (remote, backup_path), logger)
        elif options['backup']:
            logger.log('远程尚无该文件，跳过备份')

        # 2) 上传（先写临时文件再 mv，减少半截文件）
        tmp_remote = remote + '.uploading'
        logger.log('上传中...')
        sftp.put(local, tmp_remote)
        run_remote(client, 'mv -f %s %s' % (tmp_remote, remote), logger)
        uploaded = True
        logger.log('上传完成')

        # 3) 去 CRLF + 可执行权限 + 同步到 devel（roslaunch 实际加载处）
        if options['strip_crlf'] and remote.endswith('.py'):
            run_remote(client, "sed -i 's/\\r$//' %s" % remote, logger)
        run_remote(client, 'chmod +x %s' % remote, logger, check=False)
        sync_cmd = (
            'bash -lc \''
            'for d in ~/ros_ws/devel/lib/hiwonder_peripherals '
            '~/ros_ws/install/lib/hiwonder_peripherals; do '
            '  if [ -d \"$d\" ]; then cp -a %s \"$d/$(basename %s)\"; chmod +x \"$d/$(basename %s)\"; fi; '
            'done\''
        ) % (remote, remote, remote)
        if 'hiwonder_peripherals/scripts/' in remote:
            run_remote(client, sync_cmd, logger, check=False)

        # 4) 校验
        st = sftp.stat(remote)
        if st.st_size != local_size:
            raise RuntimeError('大小不一致: local=%d remote=%d' % (local_size, st.st_size))
        logger.log('大小校验通过: %d' % st.st_size)

        if options['verify_md5']:
            remote_md5 = md5_remote(sftp, remote)
            if remote_md5 != local_md5:
                raise RuntimeError('MD5 不一致: local=%s remote=%s' % (local_md5, remote_md5))
            logger.log('MD5 校验通过: %s' % remote_md5)

        # 5) 可选重启手柄（分步执行；pkill 用 [j] 避免误杀当前 shell）
        if do_restart and options['restart_joystick']:
            env = (
                'source ~/ros_ws/devel/setup.bash >/dev/null 2>&1 || true; '
                'source ~/.hiwonderrc >/dev/null 2>&1 || true; '
                'export ROS_MASTER_URI=http://127.0.0.1:11311; '
                'export ROS_IP=127.0.0.1; '
                'unset ROS_HOSTNAME; '
                'export MACHINE_TYPE=${MACHINE_TYPE:-JetRover_Mecanum}; '
                'export OPENBLAS_CORETYPE=${OPENBLAS_CORETYPE:-ARMV8}; '
            )
            run_remote(client, "bash -c '%s rosnode kill /joystick_control >/dev/null 2>&1 || true'" % env,
                       logger, check=False)
            time.sleep(1)
            run_remote(client, "pkill -f '[j]oystick_control.py' >/dev/null 2>&1 || true",
                       logger, check=False)
            time.sleep(1)
            run_remote(
                client,
                "bash -c '%s nohup python %s >/tmp/joystick_control_run.log 2>&1 & sleep 3; "
                "rosnode list 2>/dev/null | grep joystick || echo joystick_not_listed'"
                % (env, remote),
                logger, check=False)
            logger.log('已尝试重启 /joystick_control')


        logger.log('成功')
        return 0

    except Exception as e:
        logger.log('失败: %s' % e)
        logger.log(traceback.format_exc())

        # 自动回滚
        if backup_path and client is not None:
            try:
                logger.log('开始回滚: %s -> %s' % (backup_path, remote))
                run_remote(client, 'cp -a %s %s' % (backup_path, remote), logger)
                logger.log('回滚成功，已恢复备份')
            except Exception as re:
                logger.log('回滚失败: %s' % re)
        elif uploaded and not had_original and client is not None:
            try:
                logger.log('原先无文件，删除半成品: %s' % remote)
                run_remote(client, 'rm -f %s %s.uploading' % (remote, remote), logger, check=False)
            except Exception as re:
                logger.log('清理半成品失败: %s' % re)
        else:
            logger.log('无可用备份，无法回滚（或尚未开始上传）')

        return 1

    finally:
        try:
            if sftp is not None:
                sftp.close()
        except Exception:
            pass
        try:
            if client is not None:
                client.close()
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(description='上传文件到小车（失败自动回滚）')
    parser.add_argument('--config', default=DEFAULT_CONFIG, help='ini 配置路径')
    parser.add_argument('--no-restart', action='store_true', help='上传后不重启手柄节点')
    args = parser.parse_args()

    os.makedirs(LOG_DIR, exist_ok=True)
    log_name = 'upload_%s.txt' % datetime.now().strftime('%Y%m%d_%H%M%S')
    log_path = os.path.join(LOG_DIR, log_name)
    logger = TeeLogger(log_path)
    logger.log('日志文件: %s' % log_path)
    logger.log('配置文件: %s' % args.config)

    try:
        car, files, options = load_config(args.config)
        # 日志里不打印明文密码
        logger.log('目标: %s@%s:%s' % (car['user'], car['host'], car['port']))
        code = upload_with_rollback(
            car, files, options, logger, do_restart=not args.no_restart)
    except Exception as e:
        logger.log('启动失败: %s' % e)
        logger.log(traceback.format_exc())
        code = 1
    finally:
        logger.close()

    print('完整日志: %s' % log_path)
    return code


if __name__ == '__main__':
    sys.exit(main())
