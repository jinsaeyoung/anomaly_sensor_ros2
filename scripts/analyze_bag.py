#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rosbag 데이터 분석 스크립트

사용법:
  analyze_drone <bag_경로>

기능:
  1. mavros 원본 메시지 파싱 (쿼터니언→오일러, 단위 변환 등)
  2. 토픽별 CSV 저장
  3. 10Hz 기준 정렬 통합 merged CSV 생성
  4. 기본 통계 + 그래프 출력

결과 저장 위치:
  ~/anomaly_data/analyzed/<bag이름>/
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

LOCAL_TZ       = 'Asia/Seoul'   # 로컬 타임존
RESAMPLE_HZ    = 10             # 최종 merged CSV 정렬 주기
RESAMPLE_MS    = int(1000 / RESAMPLE_HZ)  # 100ms


def quat_to_euler(x, y, z, w):
    roll  = math.degrees(math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2*(w*y - z*x)))))
    yaw   = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
    return roll, pitch, yaw


def parse_msg(topic, msg):
    """mavros 원본 메시지 → flat dict"""
    row = {}

    if topic == '/mavros/imu/data':
        a = msg.linear_acceleration
        row.update({'IMU_AccX': a.x, 'IMU_AccY': a.y, 'IMU_AccZ': a.z})
        g = msg.angular_velocity
        row.update({'RATE_R': math.degrees(g.x), 'RATE_P': math.degrees(g.y), 'RATE_Y': math.degrees(g.z)})
        q = msg.orientation
        roll, pitch, yaw = quat_to_euler(q.x, q.y, q.z, q.w)
        row.update({'ATT_Roll': roll, 'ATT_Pitch': pitch, 'ATT_Yaw': yaw})

    elif topic == '/mavros/imu/data_raw':
        g = msg.angular_velocity
        row.update({'IMU_GyrX': math.degrees(g.x), 'IMU_GyrY': math.degrees(g.y), 'IMU_GyrZ': math.degrees(g.z)})
        a = msg.linear_acceleration
        row.update({'IMU_AccX_raw': a.x, 'IMU_AccY_raw': a.y, 'IMU_AccZ_raw': a.z})

    elif topic == '/mavros/imu/mag':
        m = msg.magnetic_field
        row.update({'Mag_X': m.x, 'Mag_Y': m.y, 'Mag_Z': m.z})

    elif topic == '/mavros/setpoint_raw/target_attitude':
        q = msg.orientation
        roll, pitch, yaw = quat_to_euler(q.x, q.y, q.z, q.w)
        row.update({'ATT_DesRoll': roll, 'ATT_DesPitch': pitch, 'ATT_DesYaw': yaw})
        r = msg.body_rate
        row.update({'RATE_RDes': math.degrees(r.x), 'RATE_PDes': math.degrees(r.y), 'RATE_YDes': math.degrees(r.z)})

    elif topic == '/mavros/setpoint_raw/target_local':
        p = msg.position
        row.update({'DesPos_X': p.x, 'DesPos_Y': p.y, 'DesPos_Z': p.z})
        v = msg.velocity
        row.update({'DesVel_X': v.x, 'DesVel_Y': v.y, 'DesVel_Z': v.z})

    elif topic == '/mavros/battery':
        row.update({'BAT_Volt': msg.voltage, 'BAT_Curr': msg.current})
        if hasattr(msg, 'charge') and msg.charge > 0:
            row['BAT_CurrTot'] = msg.charge

    elif topic == '/mavros/rc/out':
        for i in range(min(8, len(msg.channels))):
            row[f'RCOU_C{i+1}'] = msg.channels[i]

    elif topic == '/mavros/vibration/raw/vibration':
        v = msg.vibration
        row.update({'VIBE_X': v.x, 'VIBE_Y': v.y, 'VIBE_Z': v.z})

    elif topic == '/mavros/global_position/raw/fix':
        row.update({'GPS_Lat': msg.latitude, 'GPS_Lon': msg.longitude, 'GPS_Alt': msg.altitude})

    elif topic == '/mavros/global_position/raw/gps_vel':
        v = msg.twist.linear
        spd = math.sqrt(v.x**2 + v.y**2)
        crs = math.degrees(math.atan2(v.y, v.x))
        row.update({'GPS_GroundSpeed': spd, 'GPS_CourseAngle': crs})

    elif topic == '/mavros/local_position/pose':
        p = msg.pose.position
        row.update({'LocalNED_X': p.x, 'LocalNED_Y': p.y, 'LocalNED_Z': p.z})

    elif topic == '/mavros/local_position/velocity_local':
        v = msg.twist.linear
        row.update({'LocalNED_VX': v.x, 'LocalNED_VY': v.y, 'LocalNED_VZ': v.z})

    elif topic == '/mavros/vfr_hud':
        row.update({'VFR_GroundSpeed': msg.groundspeed, 'VFR_Alt': msg.altitude,
                    'VFR_Climb': msg.climb, 'VFR_Heading': msg.heading})

    elif topic == '/mavros/state':
        row.update({'State_Armed': int(msg.armed), 'State_Mode': msg.mode,
                    'State_Connected': int(msg.connected)})

    elif topic == '/thl100/data':
        try:
            d = json.loads(msg.data)
            row.update({'THL100_Temp': d.get('temperature'), 'THL100_Humi': d.get('humidity'),
                        'THL100_Light': d.get('light'), 'THL100_Valid': int(d.get('valid', False))})
        except Exception:
            pass

    elif topic == '/thl100/raw':
        row['THL100_Raw'] = msg.data

    elif topic == '/wcm6800/data':
        try:
            d = json.loads(msg.data)
            row.update({'WCM_Current': d.get('current'), 'WCM_Type': d.get('current_type'),
                        'WCM_Valid': int(d.get('valid', False))})
        except Exception:
            pass

    elif topic == '/wcm6800/raw':
        row['WCM_Raw'] = msg.data

    elif topic == '/respeaker/doa':
        row['MIC_DoA'] = msg.data
    elif topic == '/respeaker/vad':
        row['MIC_VAD'] = float(msg.data)
    elif topic == '/respeaker/energy':
        row['MIC_Energy'] = msg.data

    return row


def read_bag(bag_path):
    db_path = None
    for f in os.listdir(bag_path):
        if f.endswith('.db3'):
            db_path = os.path.join(bag_path, f)
            break
    if db_path is None:
        raise FileNotFoundError(f'.db3 파일을 찾을 수 없습니다: {bag_path}')

    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    cursor.execute("SELECT id, name, type FROM topics")
    topics = {row[0]: (row[1], row[2]) for row in cursor.fetchall()}

    data = defaultdict(list)
    cursor.execute("SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp")

    for topic_id, timestamp, raw in cursor.fetchall():
        topic_name, topic_type = topics[topic_id]
        try:
            msg_class = get_message(topic_type)
            msg = deserialize_message(raw, msg_class)
        except Exception:
            continue

        row = parse_msg(topic_name, msg)
        if row:
            row['timestamp'] = timestamp
            data[topic_name].append(row)

    conn.close()

    all_ts = [r['timestamp'] for rows in data.values() for r in rows]
    if not all_ts:
        return {}, 0
    bag_start = min(all_ts)

    dfs = {}
    for topic, rows in data.items():
        df = pd.DataFrame(rows)
        if 'timestamp' in df.columns:
            df['time_sec'] = (df['timestamp'] - bag_start) / 1e9
            dt_utc = pd.to_datetime(df['timestamp'], unit='ns', utc=True)
            df['datetime'] = dt_utc.dt.tz_convert(LOCAL_TZ).dt.tz_localize(None)
        dfs[topic] = df

    return dfs, bag_start


def save_csv(dfs, output_prefix):
    out_dir = f'{output_prefix}_csv'
    os.makedirs(out_dir, exist_ok=True)
    for topic, df in dfs.items():
        safe_name = topic.strip('/').replace('/', '_')
        path = os.path.join(out_dir, f'{safe_name}.csv')
        df.to_csv(path, index=False)
        print(f'  저장: {path} ({len(df)} rows)')


def merge_10hz(dfs, output_prefix, bag_start_ns):
    """
    모든 토픽을 10Hz 기준 시간축으로 정렬
    - 연속값: 최근값 유지 (forward fill)
    - 미래 데이터 사용 금지 (ffill만 사용)
    - 각 값의 age_ms 추가 (얼마나 오래된 값인지)
    """
    print(f'\n{RESAMPLE_HZ}Hz 기준 정렬 중...')

    # 전체 지속시간 계산
    all_times = []
    for df in dfs.values():
        if 'time_sec' in df.columns and len(df) > 0:
            all_times.extend(df['time_sec'].tolist())
    if not all_times:
        print('  데이터가 없습니다.')
        return None

    duration = max(all_times)
    n_steps = int(duration * RESAMPLE_HZ) + 1
    time_grid = np.arange(n_steps) * (1.0 / RESAMPLE_HZ)  # 0, 0.1, 0.2, ...

    merged = pd.DataFrame({'time_sec': time_grid})

    # 토픽별로 컬럼 추가
    str_topics = {'/mavros/state', '/thl100/raw', '/wcm6800/raw'}

    for topic, df in dfs.items():
        if 'time_sec' not in df.columns or len(df) == 0:
            continue

        df_sorted = df.sort_values('time_sec').reset_index(drop=True)
        value_cols = [c for c in df_sorted.columns
                      if c not in ('timestamp', 'time_sec', 'datetime')]

        for col in value_cols:
            if col in ('THL100_Raw', 'WCM_Raw', 'State_Mode'):
                # 문자열 컬럼은 merge_asof로 최근값 유지
                sub = df_sorted[['time_sec', col]].dropna()
                if len(sub) == 0:
                    continue
                merged = pd.merge_asof(
                    merged, sub.rename(columns={col: col}),
                    on='time_sec', direction='backward'
                )
            else:
                # 숫자 컬럼
                sub = df_sorted[['time_sec', col]].dropna()
                if len(sub) == 0:
                    continue
                merged = pd.merge_asof(
                    merged, sub,
                    on='time_sec', direction='backward'
                )

    # datetime, time_ms 컬럼 추가
    dt_utc = pd.to_datetime(
        bag_start_ns + (merged['time_sec'] * 1e9).astype('int64'),
        unit='ns', utc=True
    )
    merged.insert(0, 'time_ms',  (merged['time_sec'] * 1000).round(1))
    merged.insert(0, 'datetime', dt_utc.dt.tz_convert(LOCAL_TZ).dt.tz_localize(None))

    out_path = f'{output_prefix}_merged_{RESAMPLE_HZ}hz.csv'
    merged.to_csv(out_path, index=False)
    print(f'  병합 CSV 저장: {out_path} ({len(merged)} rows, {len(merged.columns)} cols)')
    return merged


def print_summary(dfs):
    print('\n' + '=' * 60)
    print(' 토픽별 요약')
    print('=' * 60)
    for topic, df in dfs.items():
        n = len(df)
        if n == 0:
            continue
        duration = df['time_sec'].iloc[-1] - df['time_sec'].iloc[0] if n > 1 else 0
        hz = n / duration if duration > 0 else 0
        print(f'{topic:50s} n={n:6d}  {duration:6.1f}s  ~{hz:6.1f}Hz')


def plot_key_topics(dfs, output_prefix):
    plot_map = [
        ('/mavros/imu/data',                'ATT_Roll',   'Roll (deg)'),
        ('/mavros/imu/data',                'ATT_Pitch',  'Pitch (deg)'),
        ('/mavros/imu/data',                'ATT_Yaw',    'Yaw (deg)'),
        ('/mavros/imu/data',                'IMU_AccZ',   'AccZ (m/s²)'),
        ('/mavros/battery',                 'BAT_Volt',   'Battery Volt (V)'),
        ('/mavros/battery',                 'BAT_Curr',   'Battery Curr (A)'),
        ('/mavros/vibration/raw/vibration', 'VIBE_X',     'Vibe X'),
        ('/thl100/data',                    'THL100_Temp','Temp (°C)'),
        ('/wcm6800/data',                   'WCM_Current','Current (A)'),
        ('/respeaker/energy',               'MIC_Energy', 'MIC Energy (RMS)'),
    ]

    plots = []
    for topic, col, label in plot_map:
        if topic not in dfs:
            continue
        df = dfs[topic]
        if col in df.columns and df[col].notna().sum() > 0:
            plots.append((df, col, label))

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

    bag_path     = sys.argv[1].rstrip('/')
    bag_dir      = os.path.dirname(os.path.abspath(bag_path))
    bag_name     = os.path.basename(bag_path)
    analyzed_dir = os.path.join(bag_dir, 'analyzed', bag_name)
    os.makedirs(analyzed_dir, exist_ok=True)
    output_prefix = os.path.join(analyzed_dir, bag_name)

    print(f'bag 파일 읽는 중: {bag_path}')
    dfs, bag_start = read_bag(bag_path)
    print(f'토픽 {len(dfs)}개 로드 완료')
    print(f'결과 저장 위치: {analyzed_dir}/')

    print_summary(dfs)

    print('\nCSV 저장 중...')
    save_csv(dfs, output_prefix)

    merge_10hz(dfs, output_prefix, bag_start)

    plot_key_topics(dfs, output_prefix)

    print('\n완료!')


if __name__ == '__main__':
    main()
