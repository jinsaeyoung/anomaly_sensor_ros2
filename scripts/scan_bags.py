#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
녹화된 bag 전체를 스캔해 외부 센서 데이터 누락 여부를 한 번에 점검

ros2 bag info 를 75번 돌리는 대신 db3 를 직접 읽어 빠르게 집계합니다.
(sqlite 메타데이터만 조회하므로 메시지를 역직렬화하지 않습니다)

사용법:
  python3 scan_bags.py                    # ~/anomaly_data 전체
  python3 scan_bags.py <경로>             # 특정 폴더
  python3 scan_bags.py --csv out.csv      # 결과를 CSV 로 저장
  python3 scan_bags.py --bad              # 문제 있는 bag 만 출력
"""

import os
import sys
import glob
import sqlite3
from datetime import datetime

# 점검 대상 — 외부 센서
SENSOR_GROUPS = {
    'MIC':     ['/respeaker/doa', '/respeaker/vad',
                '/respeaker/energy', '/respeaker/audio'],
    'THL100':  ['/thl100/data', '/thl100/raw'],
    'WCM6800': ['/wcm6800/data', '/wcm6800/raw'],
}

# 참고용 — FC 데이터가 정상인지 대조
FC_REF = '/mavros/imu/data'


def list_db_files(bag_path):
    """분할된 db3 를 순서대로 반환"""
    import re
    meta = os.path.join(bag_path, 'metadata.yaml')
    files = []
    if os.path.isfile(meta):
        try:
            import yaml
            info = yaml.safe_load(open(meta)).get('rosbag2_bagfile_information', {})
            for rel in info.get('relative_file_paths', []):
                p = os.path.join(bag_path, os.path.basename(rel))
                if os.path.isfile(p):
                    files.append(p)
        except Exception:
            files = []
    if not files:
        def seq(n):
            m = re.search(r'_(\d+)\.db3$', n)
            return int(m.group(1)) if m else 0
        files = [os.path.join(bag_path, f)
                 for f in sorted(os.listdir(bag_path), key=seq)
                 if f.endswith('.db3')]
    return files


def scan_bag(bag_path):
    """
    bag 하나의 토픽별 메시지 수를 집계

    반환: {
      'name', 'start', 'duration', 'counts': {topic: n}, 'error'
    }
    """
    result = {
        'name': os.path.basename(bag_path),
        'start': None, 'duration': 0.0,
        'counts': {}, 'error': None,
    }

    try:
        db_files = list_db_files(bag_path)
    except Exception as e:
        result['error'] = f'파일 목록 실패: {e}'
        return result

    if not db_files:
        result['error'] = 'db3 없음'
        return result

    counts = {}
    t_min = t_max = None

    for db in db_files:
        try:
            conn = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
            cur = conn.cursor()

            # 토픽 id → 이름
            cur.execute("SELECT id, name FROM topics")
            topics = dict(cur.fetchall())

            # 토픽별 메시지 수 (역직렬화 없이 집계)
            cur.execute("SELECT topic_id, COUNT(*) FROM messages GROUP BY topic_id")
            for tid, n in cur.fetchall():
                name = topics.get(tid)
                if name:
                    counts[name] = counts.get(name, 0) + n

            # 전체 시간 범위
            cur.execute("SELECT MIN(timestamp), MAX(timestamp) FROM messages")
            lo, hi = cur.fetchone()
            if lo:
                t_min = lo if t_min is None else min(t_min, lo)
                t_max = hi if t_max is None else max(t_max, hi)

            # 선언만 되고 데이터가 없는 토픽도 0 으로 기록
            for name in topics.values():
                counts.setdefault(name, 0)

            conn.close()
        except Exception as e:
            result['error'] = f'{os.path.basename(db)}: {e}'
            continue

    result['counts'] = counts
    if t_min:
        result['start'] = datetime.fromtimestamp(t_min / 1e9)
        result['duration'] = (t_max - t_min) / 1e9
    return result


def classify(counts):
    """
    센서 그룹별 상태 판정

    반환: {group: 'OK' | 'ZERO' | 'MISSING'}
      OK      : 데이터 있음
      ZERO    : 토픽은 있으나 0건 (노드는 살아있었으나 장치 미연결)
      MISSING : 토픽 자체 없음 (녹화 시점에 노드가 죽어 있었음)
    """
    status = {}
    for group, topics in SENSOR_GROUPS.items():
        present = [t for t in topics if t in counts]
        if not present:
            status[group] = 'MISSING'
        elif sum(counts[t] for t in present) > 0:
            status[group] = 'OK'
        else:
            status[group] = 'ZERO'
    return status


def main():
    args = sys.argv[1:]
    only_bad = '--bad' in args
    csv_out = None
    if '--csv' in args:
        csv_out = args[args.index('--csv') + 1]

    paths = [a for a in args if not a.startswith('--') and a != csv_out]
    root = paths[0] if paths else os.path.expanduser('~/anomaly_data')

    # bag 폴더 수집 (metadata.yaml 또는 db3 가 있는 디렉토리)
    bags = []
    for d in sorted(glob.glob(os.path.join(root, '*'))):
        if not os.path.isdir(d):
            continue
        if os.path.basename(d) == 'analyzed':
            continue
        if os.path.exists(os.path.join(d, 'metadata.yaml')) or \
           glob.glob(os.path.join(d, '*.db3')):
            bags.append(d)

    if not bags:
        print(f'bag 을 찾을 수 없습니다: {root}')
        sys.exit(1)

    print(f'스캔 대상: {len(bags)}개  ({root})')
    print()
    print(f'{"bag 이름":38} {"시작시각":17} {"길이":>7}  {"MIC":>7} {"THL":>7} {"WCM":>7}  {"FC":>6}')
    print('-' * 104)

    rows = []
    stat = {'all_ok': 0, 'partial': 0, 'none': 0, 'error': 0}

    for b in bags:
        r = scan_bag(b)
        if r['error'] and not r['counts']:
            print(f'{r["name"]:38} ERROR: {r["error"]}')
            stat['error'] += 1
            continue

        st = classify(r['counts'])
        fc = r['counts'].get(FC_REF, 0)
        start = r['start'].strftime('%m-%d %H:%M:%S') if r['start'] else '-'

        def mark(s):
            return {'OK': 'OK', 'ZERO': '0건', 'MISSING': '없음'}[s]

        bad = any(v != 'OK' for v in st.values())
        if not bad:
            stat['all_ok'] += 1
        elif all(v != 'OK' for v in st.values()):
            stat['none'] += 1
        else:
            stat['partial'] += 1

        rows.append({
            'name': r['name'], 'start': start,
            'duration': round(r['duration'], 1),
            'mic': st['MIC'], 'thl': st['THL100'], 'wcm': st['WCM6800'],
            'fc': fc,
        })

        if only_bad and not bad:
            continue

        flag = '  <==' if bad else ''
        print(f'{r["name"]:38} {start:17} {r["duration"]:6.1f}s  '
              f'{mark(st["MIC"]):>7} {mark(st["THL100"]):>7} {mark(st["WCM6800"]):>7}  '
              f'{fc:6d}{flag}')

    print('-' * 104)
    print(f'정상 {stat["all_ok"]}개 / 일부누락 {stat["partial"]}개 / '
          f'전체누락 {stat["none"]}개 / 오류 {stat["error"]}개')
    print()
    print('  OK   = 데이터 있음')
    print('  0건  = 토픽은 있으나 데이터 0 (노드는 동작, 장치 미연결)')
    print('  없음 = 토픽 자체 없음 (녹화 시점에 노드가 죽어 있었음)')

    if csv_out:
        import csv
        with open(csv_out, 'w', newline='', encoding='utf-8') as f:
            w = csv.DictWriter(f, fieldnames=['name', 'start', 'duration',
                                              'mic', 'thl', 'wcm', 'fc'])
            w.writeheader()
            w.writerows(rows)
        print(f'\nCSV 저장: {csv_out}')


if __name__ == '__main__':
    main()
