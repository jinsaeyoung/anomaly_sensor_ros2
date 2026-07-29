#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rosbag 데이터 분석 스크립트

사용법:
  analyze_drone <bag_경로>

기능:
  1. mavros 원본 메시지 파싱 (header.stamp 우선 사용)
  2. 좌표계 명시 (MAVROS는 ENU, 필요 시 NED 변환 컬럼 제공)
  3. 토픽별 CSV 저장
  4. 10Hz 기준 정렬 + 센서별 age_ms / stale 처리
  5. 기본 통계 + 그래프 출력

결과 저장 위치:
  <bag폴더>/analyzed/<bag이름>/
"""

import sys
import os
import math
import json
import sqlite3
from collections import defaultdict

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

# ══════════════════════════════════════════════════════════════════════════════
# 설정
# ══════════════════════════════════════════════════════════════════════════════
LOCAL_TZ    = 'Asia/Seoul'   # 로컬 타임존
RESAMPLE_HZ = 10             # merged CSV 정렬 주기

# header.stamp 신뢰 임계값 (초)
#   FC의 GPS fix가 없으면 ArduPilot이 부팅 후 경과시간을 타임스탬프로 사용하여
#   시스템 시각과 수 시간까지 어긋날 수 있습니다.
#   rosbag 기록 시각과 이 값 이상 차이나면 header.stamp를 버리고 bag_time을 사용합니다.
MAX_STAMP_SKEW_SEC = 5.0

# 센서별 허용 최대 age (초). 초과 시 NaN 처리 + stale 플래그
STALE_LIMITS = {
    'THL100':  3.0,   # 1Hz 센서 → 3초
    'WCM':     1.0,   # 10Hz 센서 → 1초
    'GPS':     2.0,
    'BAT':     2.0,
    'State':   5.0,
    'IMU':     0.5,
    'ATT':     0.5,
    'RATE':    0.5,
    'RCOU':    1.0,
    'VIBE':    2.0,
    'MIC':     1.0,
    'Mag':     1.0,
    'LocalENU': 1.0,
    'VFR':     1.0,
    'Des':     1.0,
}
DEFAULT_STALE = 2.0

# merged CSV에서 제외할 컬럼 (문자열 원시 패킷 등)
EXCLUDE_FROM_MERGED = {'THL100_Raw', 'WCM_Raw', 'stamp_source'}


def quat_to_euler(x, y, z, w):
    """쿼터니언 → 오일러(deg)"""
    roll  = math.degrees(math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2*(w*y - z*x)))))
    yaw   = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
    return roll, pitch, yaw


def get_source_time_ns(topic, msg, bag_time_ns):
    """
    데이터의 실제 발생 시각(ns) 추출

    우선순위: MAVROS header.stamp > UART JSON stamp > rosbag 기록 시각

    단, 추출한 시각이 rosbag 기록 시각과 MAX_STAMP_SKEW_SEC 이상 어긋나면
    신뢰할 수 없는 값으로 보고 bag_time_ns 로 대체합니다.
    (GPS fix 없을 때 FC가 부팅 후 경과시간을 타임스탬프로 쓰는 경우 대응)

    반환: (source_time_ns, source_kind)
      source_kind: 'header' | 'uart' | 'bag' | 'bag(skew)'
    """
    max_skew_ns = int(MAX_STAMP_SKEW_SEC * 1e9)

    # 1) ROS Header
    if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
        s = msg.header.stamp
        ns = int(s.sec) * 1_000_000_000 + int(s.nanosec)
        if ns > 0:
            if abs(ns - bag_time_ns) <= max_skew_ns:
                return ns, 'header'
            return bag_time_ns, 'bag(skew)'

    # 2) UART JSON 페이로드 내부 시각
    if topic in ('/thl100/data', '/wcm6800/data') and hasattr(msg, 'data'):
        try:
            d = json.loads(msg.data)
            if 'stamp_sec' in d and 'stamp_nsec' in d:
                ns = int(d['stamp_sec']) * 1_000_000_000 + int(d['stamp_nsec'])
                if ns > 0:
                    if abs(ns - bag_time_ns) <= max_skew_ns:
                        return ns, 'uart'
                    return bag_time_ns, 'bag(skew)'
        except Exception:
            pass

    # 3) Header 없는 메시지
    return bag_time_ns, 'bag'


def parse_msg(topic, msg):
    """mavros/센서 원본 메시지 → flat dict"""
    row = {}

    # ── IMU (자세 + 가속도 + 각속도) ──────────────────────────────────
    if topic == '/mavros/imu/data':
        a = msg.linear_acceleration
        row.update({'IMU_AccX': a.x, 'IMU_AccY': a.y, 'IMU_AccZ': a.z})
        g = msg.angular_velocity
        row.update({'RATE_R': math.degrees(g.x),
                    'RATE_P': math.degrees(g.y),
                    'RATE_Y': math.degrees(g.z)})
        q = msg.orientation
        roll, pitch, yaw = quat_to_euler(q.x, q.y, q.z, q.w)
        row.update({'ATT_Roll': roll, 'ATT_Pitch': pitch, 'ATT_Yaw': yaw})

    elif topic == '/mavros/imu/data_raw':
        g = msg.angular_velocity
        row.update({'IMU_GyrX': math.degrees(g.x),
                    'IMU_GyrY': math.degrees(g.y),
                    'IMU_GyrZ': math.degrees(g.z)})
        a = msg.linear_acceleration
        row.update({'IMU_AccX_raw': a.x, 'IMU_AccY_raw': a.y, 'IMU_AccZ_raw': a.z})

    elif topic == '/mavros/imu/mag':
        m = msg.magnetic_field
        row.update({'Mag_X': m.x, 'Mag_Y': m.y, 'Mag_Z': m.z})

    # ── 목표값 ────────────────────────────────────────────────────────
    elif topic == '/mavros/setpoint_raw/target_attitude':
        q = msg.orientation
        roll, pitch, yaw = quat_to_euler(q.x, q.y, q.z, q.w)
        row.update({'ATT_DesRoll': roll, 'ATT_DesPitch': pitch, 'ATT_DesYaw': yaw})
        r = msg.body_rate
        row.update({'RATE_RDes': math.degrees(r.x),
                    'RATE_PDes': math.degrees(r.y),
                    'RATE_YDes': math.degrees(r.z)})

    elif topic == '/mavros/setpoint_raw/target_local':
        p = msg.position
        row.update({'Des_ENU_X': p.x, 'Des_ENU_Y': p.y, 'Des_ENU_Z': p.z})
        v = msg.velocity
        row.update({'Des_ENU_VX': v.x, 'Des_ENU_VY': v.y, 'Des_ENU_VZ': v.z})

    # ── 배터리 ────────────────────────────────────────────────────────
    elif topic == '/mavros/battery':
        row.update({'BAT_Volt': msg.voltage, 'BAT_Curr': msg.current})
        if hasattr(msg, 'charge') and not math.isnan(msg.charge) and msg.charge > 0:
            row['BAT_CurrTot'] = msg.charge
        if hasattr(msg, 'percentage') and not math.isnan(msg.percentage):
            row['BAT_Percent'] = msg.percentage

    # ── 모터 PWM ──────────────────────────────────────────────────────
    elif topic == '/mavros/rc/out':
        for i in range(min(8, len(msg.channels))):
            row[f'RCOU_C{i+1}'] = msg.channels[i]

    # ── 진동 ──────────────────────────────────────────────────────────
    elif topic == '/mavros/vibration/raw/vibration':
        v = msg.vibration
        row.update({'VIBE_X': v.x, 'VIBE_Y': v.y, 'VIBE_Z': v.z})
        if hasattr(msg, 'clipping'):
            for i, c in enumerate(msg.clipping[:3]):
                row[f'VIBE_Clip{i}'] = c

    # ── GPS ───────────────────────────────────────────────────────────
    elif topic == '/mavros/global_position/raw/fix':
        row.update({'GPS_Lat': msg.latitude,
                    'GPS_Lon': msg.longitude,
                    'GPS_Alt': msg.altitude,
                    'GPS_Status': msg.status.status})

    elif topic == '/mavros/global_position/raw/gps_vel':
        # MAVROS twist는 ENU 기준 (x=East, y=North, z=Up)
        v = msg.twist.linear
        spd = math.sqrt(v.x**2 + v.y**2)
        # 항공 course: 북쪽 0°, 시계방향 → atan2(East, North)
        crs = (math.degrees(math.atan2(v.x, v.y)) + 360.0) % 360.0
        row.update({'GPS_GroundSpeed': spd, 'GPS_CourseAngle': crs})

    # ── 로컬 위치/속도 (MAVROS는 ENU 좌표계) ──────────────────────────
    elif topic == '/mavros/local_position/pose':
        p = msg.pose.position
        row.update({'LocalENU_X': p.x, 'LocalENU_Y': p.y, 'LocalENU_Z': p.z})
        # NED 변환 컬럼 함께 제공
        row.update({'LocalNED_N': p.y, 'LocalNED_E': p.x, 'LocalNED_D': -p.z})

    elif topic == '/mavros/local_position/velocity_local':
        v = msg.twist.linear
        row.update({'LocalENU_VX': v.x, 'LocalENU_VY': v.y, 'LocalENU_VZ': v.z})
        row.update({'LocalNED_VN': v.y, 'LocalNED_VE': v.x, 'LocalNED_VD': -v.z})

    # ── HUD / 상태 ────────────────────────────────────────────────────
    elif topic == '/mavros/vfr_hud':
        row.update({'VFR_GroundSpeed': msg.groundspeed,
                    'VFR_Alt': msg.altitude,
                    'VFR_Climb': msg.climb,
                    'VFR_Heading': msg.heading})

    elif topic == '/mavros/state':
        row.update({'State_Armed': int(msg.armed),
                    'State_Mode': msg.mode,
                    'State_Connected': int(msg.connected)})

    # ── UART 센서 ─────────────────────────────────────────────────────
    elif topic == '/thl100/data':
        try:
            d = json.loads(msg.data)
            row.update({'THL100_Temp':  d.get('temperature'),
                        'THL100_Humi':  d.get('humidity'),
                        'THL100_Light': d.get('light'),
                        'THL100_Seq':   d.get('sequence')})
        except Exception:
            pass

    elif topic == '/thl100/raw':
        row['THL100_Raw'] = msg.data

    elif topic == '/wcm6800/data':
        try:
            d = json.loads(msg.data)
            row.update({'WCM_Current': d.get('current'),
                        'WCM_Type':    d.get('current_type')})
        except Exception:
            pass

    elif topic == '/wcm6800/raw':
        row['WCM_Raw'] = msg.data

    # ── 마이크 ────────────────────────────────────────────────────────
    elif topic == '/respeaker/doa':
        row['MIC_DoA'] = msg.data
    elif topic == '/respeaker/vad':
        row['MIC_VAD'] = float(msg.data)
    elif topic == '/respeaker/energy':
        row['MIC_Energy'] = msg.data

    return row


def read_bag(bag_path):
    """bag 읽기 — bag_time / source_time 모두 보존"""
    db_path = None
    for f in sorted(os.listdir(bag_path)):
        if f.endswith('.db3'):
            db_path = os.path.join(bag_path, f)
            break
    if db_path is None:
        raise FileNotFoundError(f'.db3 파일을 찾을 수 없습니다: {bag_path}')

    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    cursor.execute("SELECT id, name, type FROM topics")
    topics = {r[0]: (r[1], r[2]) for r in cursor.fetchall()}

    data = defaultdict(list)
    skew_stat = defaultdict(int)   # 토픽별 skew fallback 횟수
    cursor.execute("SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp")

    for topic_id, bag_ts, raw in cursor.fetchall():
        topic_name, topic_type = topics[topic_id]
        try:
            msg_class = get_message(topic_type)
            msg = deserialize_message(raw, msg_class)
        except Exception:
            continue

        row = parse_msg(topic_name, msg)
        if not row:
            continue

        src_ts, src_kind = get_source_time_ns(topic_name, msg, bag_ts)
        if src_kind == 'bag(skew)':
            skew_stat[topic_name] += 1

        row['bag_time_ns']    = bag_ts
        row['source_time_ns'] = src_ts
        row['stamp_source']   = src_kind
        data[topic_name].append(row)

    conn.close()

    # skew 발생 토픽 경고
    if skew_stat:
        print(f'\n[경고] header.stamp 가 rosbag 기록 시각과 '
              f'{MAX_STAMP_SKEW_SEC}초 이상 어긋나 bag_time 으로 대체한 토픽:')
        for t, n in sorted(skew_stat.items(), key=lambda x: -x[1]):
            total = len(data[t])
            print(f'  {t:46s} {n}/{total} 건')
        print('  (FC의 GPS fix 없음 → ArduPilot 부팅 후 경과시간을 '
              'timestamp로 사용하는 경우 발생)')

    all_src = [r['source_time_ns'] for rows in data.values() for r in rows]
    if not all_src:
        return {}, 0

    t0 = min(all_src)

    dfs = {}
    for topic, rows in data.items():
        df = pd.DataFrame(rows)
        # 원본 발생 시각 기준 상대시간
        df['time_sec'] = (df['source_time_ns'] - t0) / 1e9
        # 전송 지연 (rosbag 기록 시각 - 원본 발생 시각)
        df['transport_delay_ms'] = (df['bag_time_ns'] - df['source_time_ns']) / 1e6
        dt_utc = pd.to_datetime(df['source_time_ns'], unit='ns', utc=True)
        df['datetime'] = dt_utc.dt.tz_convert(LOCAL_TZ).dt.tz_localize(None)
        dfs[topic] = df.sort_values('time_sec').reset_index(drop=True)

    return dfs, t0


def save_csv(dfs, output_prefix):
    out_dir = f'{output_prefix}_csv'
    os.makedirs(out_dir, exist_ok=True)
    for topic, df in dfs.items():
        safe_name = topic.strip('/').replace('/', '_')
        path = os.path.join(out_dir, f'{safe_name}.csv')
        df.to_csv(path, index=False)
        print(f'  저장: {path} ({len(df)} rows)')


def _stale_limit_for(col):
    """컬럼 접두어로 허용 age 결정"""
    for prefix, limit in STALE_LIMITS.items():
        if col.startswith(prefix):
            return limit
    return DEFAULT_STALE


def merge_10hz(dfs, output_prefix, t0_ns):
    """
    10Hz 기준 정렬
    - backward merge_asof (미래 데이터 사용 금지)
    - 컬럼 그룹별 age_ms 계산, 허용 초과 시 NaN + stale 플래그
    - 원시 문자열 컬럼 제외
    """
    print(f'\n{RESAMPLE_HZ}Hz 기준 정렬 중...')

    all_times = []
    for df in dfs.values():
        if len(df) > 0:
            all_times.extend(df['time_sec'].tolist())
    if not all_times:
        print('  데이터가 없습니다.')
        return None

    duration = max(all_times)

    # 비정상적으로 긴 duration 방어 (타임스탬프 이상 시 행 수 폭증 방지)
    MAX_REASONABLE_SEC = 6 * 3600   # 6시간
    if duration > MAX_REASONABLE_SEC:
        print(f'  [경고] 계산된 duration이 {duration:.0f}초로 비정상적으로 깁니다.')
        print(f'         일부 토픽의 타임스탬프가 어긋났을 가능성이 있습니다.')
        print(f'         중앙값 기준으로 재계산합니다.')
        # 각 토픽의 최대 time_sec 중 중앙값을 기준으로 사용
        per_topic_max = sorted(
            df['time_sec'].max() for df in dfs.values() if len(df) > 0
        )
        duration = per_topic_max[len(per_topic_max) // 2]
        print(f'         → duration = {duration:.1f}초')

    n_steps  = int(duration * RESAMPLE_HZ) + 1
    grid     = np.arange(n_steps) / RESAMPLE_HZ
    merged   = pd.DataFrame({'time_sec': grid})

    # 그룹(접두어)별로 age를 한 번만 계산하기 위해 추적
    group_age_done = set()

    for topic, df in dfs.items():
        if len(df) == 0:
            continue

        value_cols = [c for c in df.columns
                      if c not in ('bag_time_ns', 'source_time_ns', 'time_sec',
                                   'datetime', 'transport_delay_ms', 'stamp_source')
                      and c not in EXCLUDE_FROM_MERGED]
        if not value_cols:
            continue

        # 이 토픽의 대표 그룹명 (첫 컬럼 접두어 기준)
        sample_col = value_cols[0]
        group = sample_col.split('_')[0]

        sub = df[['time_sec'] + value_cols].copy()
        # 해당 토픽의 마지막 수신 시각 추적용
        sub['_last_seen'] = sub['time_sec']

        merged = pd.merge_asof(
            merged.sort_values('time_sec'),
            sub.sort_values('time_sec'),
            on='time_sec',
            direction='backward'   # 미래 데이터 사용 금지
        )

        # age_ms 계산
        age_col = f'{group}_age_ms'
        if age_col not in group_age_done:
            merged[age_col] = (merged['time_sec'] - merged['_last_seen']) * 1000.0
            group_age_done.add(age_col)

        # stale 판정 및 NaN 처리
        limit = _stale_limit_for(sample_col)
        stale_mask = (merged['time_sec'] - merged['_last_seen']) > limit
        merged[f'{group}_stale'] = stale_mask.astype('Int8')

        numeric_cols = [c for c in value_cols
                        if pd.api.types.is_numeric_dtype(merged.get(c, pd.Series(dtype=float)))]
        for c in value_cols:
            if c in merged.columns:
                merged.loc[stale_mask, c] = np.nan

        merged = merged.drop(columns=['_last_seen'])

    # 시간 컬럼 정리
    dt_utc = pd.to_datetime(
        t0_ns + (merged['time_sec'] * 1e9).astype('int64'), unit='ns', utc=True
    )
    merged.insert(0, 'time_ms',  (merged['time_sec'] * 1000).round(1))
    merged.insert(0, 'datetime', dt_utc.dt.tz_convert(LOCAL_TZ).dt.tz_localize(None))

    out_path = f'{output_prefix}_merged_{RESAMPLE_HZ}hz.csv'
    merged.to_csv(out_path, index=False)
    print(f'  병합 CSV 저장: {out_path} ({len(merged)} rows, {len(merged.columns)} cols)')
    return merged


def print_summary(dfs):
    print('\n' + '=' * 92)
    print(' 토픽별 요약')
    print('=' * 92)
    print(f'{"토픽":46s} {"건수":>6s} {"구간":>8s} {"주기":>8s} {"지연":>9s}  시각출처')
    print('-' * 92)
    for topic, df in dfs.items():
        n = len(df)
        if n == 0:
            continue
        dur   = df['time_sec'].iloc[-1] - df['time_sec'].iloc[0] if n > 1 else 0
        hz    = n / dur if dur > 0 else 0
        delay = df['transport_delay_ms'].mean() if 'transport_delay_ms' in df else 0

        if 'stamp_source' in df.columns:
            kinds = df['stamp_source'].value_counts()
            src = ', '.join(f'{k}({v})' for k, v in kinds.items())
        else:
            src = '-'

        print(f'{topic:46s} {n:6d} {dur:7.1f}s {hz:7.1f}Hz {delay:8.1f}ms  {src}')


def plot_key_topics(dfs, output_prefix):
    plot_map = [
        ('/mavros/imu/data',                'ATT_Roll',    'Roll (deg)'),
        ('/mavros/imu/data',                'ATT_Pitch',   'Pitch (deg)'),
        ('/mavros/imu/data',                'ATT_Yaw',     'Yaw (deg)'),
        ('/mavros/imu/data',                'IMU_AccZ',    'AccZ (m/s^2)'),
        ('/mavros/battery',                 'BAT_Volt',    'Battery Volt (V)'),
        ('/mavros/battery',                 'BAT_Curr',    'Battery Curr (A)'),
        ('/mavros/vibration/raw/vibration', 'VIBE_X',      'Vibe X'),
        ('/thl100/data',                    'THL100_Temp', 'Temp (C)'),
        ('/wcm6800/data',                   'WCM_Current', 'Current (A)'),
        ('/respeaker/energy',               'MIC_Energy',  'MIC Energy (RMS)'),
    ]

    plots = []
    for topic, col, label in plot_map:
        if topic in dfs and col in dfs[topic].columns and dfs[topic][col].notna().sum() > 0:
            plots.append((dfs[topic], col, label))

    if not plots:
        print('\n그래프로 표시할 데이터가 없습니다.')
        return

    fig, axes = plt.subplots(len(plots), 1, figsize=(12, 2.5 * len(plots)), sharex=True)
    if len(plots) == 1:
        axes = [axes]

    for ax, (df, col, label) in zip(axes, plots):
        valid = df[['time_sec', col]].dropna()
        ax.plot(valid['time_sec'].to_numpy(), valid[col].to_numpy(), linewidth=0.8)
        ax.set_ylabel(label, fontsize=9)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    out_path = f'{output_prefix}_overview.png'
    plt.savefig(out_path, dpi=120)
    print(f'\n그래프 저장: {out_path}')
    plt.close()


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    bag_path      = sys.argv[1].rstrip('/')
    bag_dir       = os.path.dirname(os.path.abspath(bag_path))
    bag_name      = os.path.basename(bag_path)
    analyzed_dir  = os.path.join(bag_dir, 'analyzed', bag_name)
    os.makedirs(analyzed_dir, exist_ok=True)
    output_prefix = os.path.join(analyzed_dir, bag_name)

    print(f'bag 파일 읽는 중: {bag_path}')
    dfs, t0 = read_bag(bag_path)
    print(f'토픽 {len(dfs)}개 로드 완료')
    print(f'결과 저장 위치: {analyzed_dir}/')

    print_summary(dfs)

    print('\nCSV 저장 중...')
    save_csv(dfs, output_prefix)

    merge_10hz(dfs, output_prefix, t0)
    plot_key_topics(dfs, output_prefix)

    print('\n완료!')


if __name__ == '__main__':
    main()
