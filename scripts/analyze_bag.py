#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rosbag 데이터 분석 스크립트
사용법:
  python3 analyze_bag.py <bag_경로> [출력_csv_접두사]

기능:
  1. bag 파일에서 모든 토픽 메시지를 추출
  2. 토픽별 CSV 저장
  3. 타임스탬프 기준 병합 CSV 생성 (분석용 통합 테이블)
  4. 기본 통계 + 그래프 출력
"""

import sys
import os
import sqlite3
import struct
from collections import defaultdict

import pandas as pd
import matplotlib.pyplot as plt
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import yaml

LOCAL_TZ = 'Asia/Seoul'  # 데이터 수집 시스템의 로컬 타임존 — 환경에 맞게 변경


def read_bag(bag_path):
    """rosbag2 (.db3) 파일을 읽어 토픽별 DataFrame 딕셔너리 반환"""
    db_path = None
    for f in os.listdir(bag_path):
        if f.endswith('.db3'):
            db_path = os.path.join(bag_path, f)
            break
    if db_path is None:
        raise FileNotFoundError(f'.db3 파일을 찾을 수 없습니다: {bag_path}')

    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # 토픽 정보
    cursor.execute("SELECT id, name, type FROM topics")
    topics = {row[0]: (row[1], row[2]) for row in cursor.fetchall()}

    data = defaultdict(list)

    cursor.execute("SELECT topic_id, timestamp, data FROM messages")
    for topic_id, timestamp, raw in cursor.fetchall():
        topic_name, topic_type = topics[topic_id]
        try:
            msg_class = get_message(topic_type)
            msg = deserialize_message(raw, msg_class)
        except Exception:
            continue

        row = {'timestamp': timestamp}

        # 메시지 타입별 필드 추출
        if hasattr(msg, 'data'):
            val = msg.data
            if isinstance(val, (list, tuple)):
                if len(val) > 0 and isinstance(val[0], (int, float)):
                    row['value'] = val[0]  # 배열은 첫 값만 (audio 등은 분석 제외 권장)
                else:
                    row['value'] = str(val)[:50]
            else:
                row['value'] = val
        else:
            # geometry_msgs, sensor_msgs 등 복합 메시지 - 주요 필드만
            for field in ['x', 'y', 'z', 'w']:
                if hasattr(msg, field):
                    row[field] = getattr(msg, field)

        data[topic_name].append(row)

    conn.close()

    # 전체 bag 기준 시작 시각 (모든 토픽 통틀어 가장 빠른 timestamp)
    all_timestamps = [row['timestamp'] for rows in data.values() for row in rows]
    if not all_timestamps:
        return {}, 0
    bag_start = min(all_timestamps)

    dfs = {}
    for topic, rows in data.items():
        df = pd.DataFrame(rows)
        if 'timestamp' in df.columns:
            # 토픽별이 아닌 bag 전체 시작 시각 기준 — 모든 토픽이 같은 0초를 공유
            df['time_sec'] = (df['timestamp'] - bag_start) / 1e9
            # 실제 시각(사람이 읽을 수 있는 형태) — 시스템 로컬 타임존 기준
            # (rosbag 내부 timestamp는 UTC epoch이므로 tz_localize 후 변환 필요)
            dt_utc = pd.to_datetime(df['timestamp'], unit='ns', utc=True)
            df['datetime'] = dt_utc.dt.tz_convert(LOCAL_TZ).dt.tz_localize(None)
        dfs[topic] = df

    return dfs, bag_start


def save_csv(dfs, output_prefix):
    """토픽별 CSV 저장"""
    out_dir = f'{output_prefix}_csv'
    os.makedirs(out_dir, exist_ok=True)

    for topic, df in dfs.items():
        safe_name = topic.strip('/').replace('/', '_')
        path = os.path.join(out_dir, f'{safe_name}.csv')
        df.to_csv(path, index=False)
        print(f'  저장: {path} ({len(df)} rows)')

    return out_dir


def merge_by_time(dfs, output_prefix, bag_start_ns, tolerance_sec=0.5):
    """타임스탬프 기준으로 전체 토픽 병합 (asof merge)"""
    print('\n타임스탬프 기준 병합 중...')

    merged = None
    for topic, df in dfs.items():
        if 'value' not in df.columns:
            continue

        safe_name = topic.strip('/').replace('/', '_')
        sub = df[['time_sec', 'value']].copy()
        sub = sub.rename(columns={'value': safe_name})
        sub = sub.sort_values('time_sec')

        if merged is None:
            merged = sub
        else:
            merged = pd.merge_asof(
                merged.sort_values('time_sec'),
                sub,
                on='time_sec',
                tolerance=tolerance_sec,
                direction='nearest'
            )

    if merged is not None:
        # 보기 편한 시간 컬럼 추가
        merged = merged.sort_values('time_sec').reset_index(drop=True)
        merged.insert(0, 'time_ms', (merged['time_sec'] * 1000).round(1))       # 밀리초
        # bag_start(UTC epoch ns) + 경과시간 → 로컬 타임존으로 변환
        dt_utc = pd.to_datetime(
            bag_start_ns + (merged['time_sec'] * 1e9).astype('int64'), unit='ns', utc=True
        )
        merged.insert(0, 'datetime', dt_utc.dt.tz_convert(LOCAL_TZ).dt.tz_localize(None))

        out_path = f'{output_prefix}_merged.csv'
        merged.to_csv(out_path, index=False)
        print(f'  병합 CSV 저장: {out_path} ({len(merged)} rows, {len(merged.columns)} cols)')
        return merged
    else:
        print('  병합 가능한 데이터가 없습니다.')
        return None


def print_summary(dfs):
    """기본 통계 출력"""
    print('\n' + '=' * 60)
    print(' 토픽별 요약')
    print('=' * 60)
    for topic, df in dfs.items():
        n = len(df)
        if n == 0:
            continue
        duration = df['time_sec'].iloc[-1] - df['time_sec'].iloc[0] if n > 1 else 0
        hz = n / duration if duration > 0 else 0

        info = f'{topic:40s} n={n:6d}  duration={duration:6.1f}s  ~{hz:5.1f}Hz'

        if 'value' in df.columns and pd.api.types.is_numeric_dtype(df['value']):
            info += f'  min={df["value"].min():.3f} max={df["value"].max():.3f} mean={df["value"].mean():.3f}'

        print(info)


def plot_key_topics(dfs, output_prefix):
    """핵심 토픽 그래프 저장"""
    plot_topics = {
        '/drone/attitude/roll': 'Roll (deg)',
        '/drone/attitude/pitch': 'Pitch (deg)',
        '/drone/attitude/yaw': 'Yaw (deg)',
        '/drone/battery/voltage': 'Battery Voltage (V)',
        '/drone/battery/current': 'Battery Current (A)',
        '/thl100/temperature': 'Temperature (C)',
        '/thl100/humidity': 'Humidity (%)',
        '/wcm6800/current': 'Current (A)',
        '/respeaker/energy': 'Mic Energy (RMS)',
        '/respeaker/doa': 'Sound DoA (deg)',
    }

    available = [(t, l) for t, l in plot_topics.items() if t in dfs and 'value' in dfs[t].columns]
    if not available:
        print('\n그래프로 표시할 데이터가 없습니다.')
        return

    n = len(available)
    fig, axes = plt.subplots(n, 1, figsize=(12, 2.5 * n), sharex=True)
    if n == 1:
        axes = [axes]

    for ax, (topic, label) in zip(axes, available):
        df = dfs[topic]
        ax.plot(df['time_sec'].to_numpy(), df['value'].to_numpy(), linewidth=0.8)
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

    bag_path = sys.argv[1].rstrip('/')
    bag_dir = os.path.dirname(os.path.abspath(bag_path))
    bag_name = os.path.basename(bag_path)

    # output_prefix가 명시되지 않으면
    # <bag폴더의 부모>/analyzed/<bag이름>/<bag이름> 경로에 저장
    if len(sys.argv) > 2:
        output_prefix = sys.argv[2]
    else:
        analyzed_dir = os.path.join(bag_dir, 'analyzed', bag_name)
        os.makedirs(analyzed_dir, exist_ok=True)
        output_prefix = os.path.join(analyzed_dir, bag_name)

    print(f'bag 파일 읽는 중: {bag_path}')
    dfs, bag_start = read_bag(bag_path)
    print(f'토픽 {len(dfs)}개 로드 완료')
    print(f'결과 저장 위치: {os.path.dirname(output_prefix)}/')

    print_summary(dfs)

    print('\nCSV 저장 중...')
    save_csv(dfs, output_prefix)

    merge_by_time(dfs, output_prefix, bag_start)

    plot_key_topics(dfs, output_prefix)

    print('\n완료!')


if __name__ == '__main__':
    main()
