#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rosbag 에서 ReSpeaker 원본 오디오를 WAV 로 추출

/respeaker/audio 는 16kHz 6채널 int16 PCM 원본입니다.
CSV 로는 파형을 담을 수 없으므로 별도 WAV 파일로 추출합니다.

사용법:
  python3 extract_audio.py <bag_경로>              # ch0 (빔포밍 출력)
  python3 extract_audio.py <bag_경로> --all        # 전 채널 개별 WAV
  python3 extract_audio.py <bag_경로> --ch 1       # 특정 채널만

출력:
  <bag폴더>/analyzed/<bag이름>/<bag이름>_ch0.wav
"""

import sys
import os
import wave
import sqlite3

import numpy as np
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

SAMPLE_RATE  = 16000
MIC_CHANNELS = 6
SAMPLE_WIDTH = 2      # int16
TOPIC        = '/respeaker/audio'


def read_audio(bag_path):
    """bag 의 모든 split db3 에서 오디오 프레임을 시간순으로 수집"""
    db_files = sorted(f for f in os.listdir(bag_path) if f.endswith('.db3'))
    if not db_files:
        raise FileNotFoundError(f'.db3 파일이 없습니다: {bag_path}')

    frames = []
    total_msgs = 0

    for dbf in db_files:
        db_path = os.path.join(bag_path, dbf)
        conn = sqlite3.connect(db_path)
        cur = conn.cursor()

        cur.execute("SELECT id, name, type FROM topics WHERE name = ?", (TOPIC,))
        rows = cur.fetchall()
        if not rows:
            conn.close()
            continue

        topic_id, _, topic_type = rows[0]
        msg_class = get_message(topic_type)

        cur.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
            (topic_id,)
        )
        for ts, raw in cur.fetchall():
            try:
                msg = deserialize_message(raw, msg_class)
            except Exception:
                continue
            pcm = np.frombuffer(bytes(msg.data), dtype=np.int16)
            if pcm.size:
                frames.append(pcm)
                total_msgs += 1

        conn.close()
        print(f'  {dbf}: 누적 {total_msgs} 프레임')

    if not frames:
        raise ValueError(f'{TOPIC} 데이터가 없습니다. 녹화 목록에 포함되었는지 확인하세요.')

    return np.concatenate(frames)


def write_wav(path, samples):
    with wave.open(path, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(SAMPLE_WIDTH)
        w.setframerate(SAMPLE_RATE)
        w.writeframes(samples.astype(np.int16).tobytes())


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    bag_path = sys.argv[1].rstrip('/')
    args = sys.argv[2:]

    extract_all = '--all' in args
    ch_sel = 0
    if '--ch' in args:
        ch_sel = int(args[args.index('--ch') + 1])

    bag_dir  = os.path.dirname(os.path.abspath(bag_path))
    bag_name = os.path.basename(bag_path)
    out_dir  = os.path.join(bag_dir, 'analyzed', bag_name)
    os.makedirs(out_dir, exist_ok=True)

    print(f'오디오 추출 중: {bag_path}')
    interleaved = read_audio(bag_path)

    n_total = interleaved.size
    n_per_ch = n_total // MIC_CHANNELS
    duration = n_per_ch / SAMPLE_RATE

    print(f'\n총 샘플: {n_total:,} ({MIC_CHANNELS}ch interleaved)')
    print(f'채널당:  {n_per_ch:,} 샘플 = {duration:.1f}초')

    channels = range(MIC_CHANNELS) if extract_all else [ch_sel]

    print()
    for ch in channels:
        if ch >= MIC_CHANNELS:
            print(f'  채널 {ch} 없음 (0~{MIC_CHANNELS-1})')
            continue
        data = interleaved[ch::MIC_CHANNELS]
        out = os.path.join(out_dir, f'{bag_name}_ch{ch}.wav')
        write_wav(out, data)
        rms = float(np.sqrt(np.mean(data.astype(np.float32) ** 2)))
        size_mb = os.path.getsize(out) / 1024 / 1024
        print(f'  ch{ch}: {out}')
        print(f'        {size_mb:.1f} MB, RMS={rms:.1f}')

    print(f'\n완료 — {out_dir}/')
    if not extract_all:
        print('  전 채널 추출: --all 옵션 사용')


if __name__ == '__main__':
    main()
