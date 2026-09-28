#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
녹화 데이터 검증

bag 하나(또는 전체)를 읽어 "드론 데이터가 제대로 저장됐는지" 판정하고,
외부센서(THL100 · WCM6800 · ReSpeaker)가 함께 수집됐는지 표시합니다.

  판정 (필수)   mavros 드론 데이터 → 정상 / 주의 / 불량
  표기 (외부)   THL100, WCM6800, ReSpeaker → 정상 / 부분 / 없음

메시지를 역직렬화하지 않고 db3 의 수신 시각만 읽으므로 빠릅니다.
(arm 구간 확인에만 /mavros/state 를 해석하며, ROS 환경이 없으면 생략)

사용법:
  verify_bag                          # 가장 최근 bag
  verify_bag flight_20260921_190312   # 특정 bag (이름 또는 경로)
  verify_bag --all                    # 전체 bag 요약표
  verify_bag --all --csv out.csv      # 요약표를 CSV 로 저장
"""

import os
import re
import sys
import glob
import json
import signal
import sqlite3
import unicodedata


# ══════════════════════════════════════════════════════════════════════════════
# 판정 기준 — 필요하면 여기서 조정하세요
#   hz   : 기대 주기 (권장 SR 설정 기준)
#   gap  : 이 시간(초) 이상 메시지가 없으면 '끊김'
# ══════════════════════════════════════════════════════════════════════════════
DRONE_TOPICS = {
    '/mavros/imu/data':                 {'name': '자세/각속도', 'hz': 50, 'gap': 1.0},
    '/mavros/imu/data_raw':             {'name': '가속도/자이로', 'hz': 50, 'gap': 1.0},
    '/mavros/rc/out':                   {'name': '모터 PWM',    'hz': 50, 'gap': 1.0},
    '/mavros/vibration/raw/vibration':  {'name': '진동',        'hz': 20, 'gap': 2.0},
    '/mavros/battery':                  {'name': '배터리',      'hz': 10, 'gap': 3.0},
    '/mavros/state':                    {'name': '상태',        'hz': 1,  'gap': 5.0},
}

SENSOR_TOPICS = {
    'ReSpeaker': {'topic': '/respeaker/audio', 'hz': 15.6, 'gap': 2.0},
    'THL100':    {'topic': '/thl100/data',     'hz': 1.0,  'gap': 5.0},
    'WCM6800':   {'topic': '/wcm6800/data',    'hz': 10.0, 'gap': 3.0},
}

RATE_WARN = 0.7     # 기대 주기의 70% 미만이면 '주기 낮음'
BIG_GAP   = 5.0     # 드론 데이터가 이 시간 이상 끊기면 '불량'


# ══════════════════════════════════════════════════════════════════════════════
# bag 읽기
# ══════════════════════════════════════════════════════════════════════════════
def data_dir():
    return os.environ.get('ANOMALY_DATA', os.path.expanduser('~/anomaly_data'))


def list_bags(root=None):
    root = root or data_dir()
    out = []
    for d in glob.glob(os.path.join(root, '*')):
        if not os.path.isdir(d) or os.path.basename(d) == 'analyzed':
            continue
        if os.path.exists(os.path.join(d, 'metadata.yaml')) or glob.glob(os.path.join(d, '*.db3')):
            out.append(d)
    out.sort(key=os.path.getmtime)
    return out


def db_files(bag):
    """
    metadata.yaml 순서를 따르고, 없으면 파일명 숫자 순서

    metadata.yaml 은 rosbag 이 정상 종료할 때 기록됩니다.
    드론 전원이 갑자기 끊기면 이 파일이 없어 '마감되지 않은 bag' 이 됩니다.
    데이터 자체는 대개 읽을 수 있으므로, 알리기만 하고 계속 진행합니다.
    """
    files, missing = [], []
    meta = os.path.join(bag, 'metadata.yaml')
    if os.path.isfile(meta):
        try:
            import yaml
            info = yaml.safe_load(open(meta)).get('rosbag2_bagfile_information', {})
            for rel in info.get('relative_file_paths', []):
                p = os.path.join(bag, os.path.basename(rel))
                (files if os.path.isfile(p) else missing).append(p)
        except Exception:
            files = []
    if not files and not missing:
        seq = lambda n: int(m.group(1)) if (m := re.search(r'_(\d+)\.db3$', n)) else 0
        files = [os.path.join(bag, f) for f in sorted(os.listdir(bag), key=seq) if f.endswith('.db3')]
    return files, missing


def read_timestamps(bag, topics):
    """
    토픽별 수신 시각(ns) 목록과 bag 전체 시작·끝

    반환: (ts_by_topic, t_start, t_end, errors, present_topics)
    """
    ts = {t: [] for t in topics}
    present, errors = set(), []
    t0 = t1 = None
    files, missing = db_files(bag)
    errors += [f'분할 파일 없음: {os.path.basename(m)}' for m in missing]

    for db in files:
        try:
            conn = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
            cur = conn.cursor()
            cur.execute('SELECT id, name FROM topics')
            ids = {name: tid for tid, name in cur.fetchall()}
            present |= set(ids)
            cur.execute('SELECT MIN(timestamp), MAX(timestamp) FROM messages')
            lo, hi = cur.fetchone()
            if lo is not None:
                t0 = lo if t0 is None else min(t0, lo)
                t1 = hi if t1 is None else max(t1, hi)
            for t in topics:
                if t in ids:
                    cur.execute('SELECT timestamp FROM messages WHERE topic_id=? ORDER BY timestamp',
                                (ids[t],))
                    ts[t].extend(r[0] for r in cur.fetchall())
            conn.close()
        except Exception as e:
            errors.append(f'{os.path.basename(db)} 읽기 실패: {e}')

    for t in ts:
        ts[t].sort()
    return ts, t0, t1, errors, present


# ══════════════════════════════════════════════════════════════════════════════
# 분석
# ══════════════════════════════════════════════════════════════════════════════
def topic_stats(stamps, t0, t1, hz, gap):
    """
    주기와 끊김 구간 계산

    bag 시작~첫 메시지, 마지막 메시지~bag 끝도 공백으로 봅니다.
    (센서가 늦게 붙었거나 중간에 죽은 경우를 잡기 위함)

    반환: {'count','hz','rate_ok','gaps':[(시작s, 길이s)], 'max_gap'}
    """
    n = len(stamps)
    dur = max((t1 - t0) / 1e9, 1e-9) if t0 is not None else 0
    if n == 0 or dur <= 0:
        return {'count': 0, 'hz': 0.0, 'rate_ok': False, 'gaps': [], 'max_gap': dur}

    points = [t0] + stamps + [t1]
    gaps = []
    for a, b in zip(points, points[1:]):
        g = (b - a) / 1e9
        if g >= gap:
            gaps.append(((a - t0) / 1e9, g))

    # 주기는 실제 수신 구간 기준 (시작 지연·종료 후 공백은 제외)
    active = (stamps[-1] - stamps[0]) / 1e9
    measured = (n - 1) / active if active > 0 and n > 1 else 0.0
    return {
        'count': n,
        'hz': measured,
        'rate_ok': measured >= hz * RATE_WARN,
        'gaps': gaps,
        'max_gap': max((g for _, g in gaps), default=0.0),
    }


def judge_drone(stats):
    """드론 데이터 판정 — (등급, 사유 목록)"""
    reasons, level = [], '정상'
    for topic, s in stats.items():
        name = DRONE_TOPICS[topic]['name']
        if s['count'] == 0:
            reasons.append(f'{name} 없음')
            level = '불량'
        elif s['max_gap'] >= BIG_GAP:
            reasons.append(f'{name} {s["max_gap"]:.0f}초 끊김')
            level = '불량'
        else:
            if s['gaps']:
                reasons.append(f'{name} 끊김 {len(s["gaps"])}회')
                level = '주의' if level == '정상' else level
            if not s['rate_ok']:
                reasons.append(f'{name} {s["hz"]:.1f}Hz (기대 {DRONE_TOPICS[topic]["hz"]})')
                level = '주의' if level == '정상' else level
    return level, reasons


def judge_sensor(s):
    """외부센서 표기 — 정상 / 부분 / 없음"""
    if s['count'] == 0:
        return '없음'
    if s['gaps'] or not s['rate_ok']:
        return '부분'
    return '정상'


def arm_segments(bag):
    """/mavros/state 에서 arm 구간 추출 (ROS 환경이 없으면 None)"""
    try:
        from rclpy.serialization import deserialize_message
        from mavros_msgs.msg import State
    except Exception:
        return None
    segs, start, t0 = [], None, None
    files, _ = db_files(bag)
    rows = []
    for db in files:
        try:
            conn = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
            cur = conn.cursor()
            cur.execute("SELECT id FROM topics WHERE name='/mavros/state'")
            r = cur.fetchone()
            if r:
                cur.execute('SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp', (r[0],))
                rows += cur.fetchall()
            cur.execute('SELECT MIN(timestamp) FROM messages')
            lo = cur.fetchone()[0]
            if lo is not None:
                t0 = lo if t0 is None else min(t0, lo)
            conn.close()
        except Exception:
            continue
    last = None
    for ts, raw in sorted(rows):
        try:
            armed = deserialize_message(raw, State).armed
        except Exception:
            continue
        if armed and start is None:
            start = ts
        elif not armed and start is not None:
            segs.append(((start - t0) / 1e9, (ts - start) / 1e9))
            start = None
        last = ts
    if start is not None and last is not None:
        segs.append(((start - t0) / 1e9, (last - start) / 1e9))
    return segs


def read_side_files(bag):
    """_meta.json, _record.log 요약"""
    base = bag.rstrip('/')
    meta, log_issues = None, 0
    try:
        meta = json.load(open(base + '_meta.json', encoding='utf-8'))
    except Exception:
        pass
    try:
        for line in open(base + '_record.log', encoding='utf-8', errors='ignore'):
            if 'ERROR' in line or '[WARN]' in line:
                log_issues += 1
    except Exception:
        log_issues = None
    return meta, log_issues


def verify(bag, with_arm=True):
    topics = list(DRONE_TOPICS) + [v['topic'] for v in SENSOR_TOPICS.values()]
    ts, t0, t1, errors, present = read_timestamps(bag, topics)
    dur = (t1 - t0) / 1e9 if t0 is not None else 0.0

    # 전원 차단으로 마감되지 못한 bag — 데이터는 살아 있을 수 있습니다
    unclosed = not os.path.isfile(os.path.join(bag, 'metadata.yaml'))

    drone = {t: topic_stats(ts[t], t0, t1, c['hz'], c['gap']) for t, c in DRONE_TOPICS.items()}
    sensors = {g: topic_stats(ts[c['topic']], t0, t1, c['hz'], c['gap']) for g, c in SENSOR_TOPICS.items()}
    level, reasons = judge_drone(drone)
    if unclosed:
        # 마지막 몇 초가 잘렸을 수 있으므로 '주의' 로 낮춥니다
        level = '불량' if level == '불량' else '주의'
        reasons = ['마감되지 않음(전원 차단 추정)'] + reasons
    if errors:
        level = '불량'
        reasons = errors + reasons
    meta, log_issues = read_side_files(bag)
    return {
        'bag': bag, 'name': os.path.basename(bag.rstrip('/')),
        'duration': dur, 'errors': errors, 'unclosed': unclosed,
        'drone': drone, 'drone_level': level, 'drone_reasons': reasons,
        'sensors': sensors, 'sensor_state': {g: judge_sensor(s) for g, s in sensors.items()},
        'arm': arm_segments(bag) if with_arm else None,
        'meta': meta, 'log_issues': log_issues,
    }


# ══════════════════════════════════════════════════════════════════════════════
# 출력
# ══════════════════════════════════════════════════════════════════════════════
def _pad(s, width):
    """한글(2칸) 을 고려해 오른쪽을 공백으로 채움"""
    s = str(s)
    w = sum(2 if unicodedata.east_asian_width(ch) in ('W', 'F') else 1 for ch in s)
    return s + ' ' * max(0, width - w)


MARK = {'정상': '✅', '주의': '⚠ ', '불량': '❌', '부분': '⚠ ', '없음': '❌'}


def print_one(r):
    arm = r['arm']
    arm_s = ''
    if arm is not None:
        fly = sum(d for _, d in arm)
        arm_s = f'   비행 {fly:.0f}초 (arm {len(arm)}회)' if arm else '   지상 녹화 (arm 없음)'
    print(f"\n{r['name']}   길이 {r['duration']:.0f}초{arm_s}")
    print('─' * 64)

    # 드론 (판정)
    print(f"  드론 데이터   {MARK[r['drone_level']]} {r['drone_level']}")
    for topic, s in r['drone'].items():
        c = DRONE_TOPICS[topic]
        gap = f"  끊김 {len(s['gaps'])}회(최대 {s['max_gap']:.1f}초)" if s['gaps'] else ''
        low = '' if s['rate_ok'] or s['count'] == 0 else '  주기 낮음'
        state = '없음' if s['count'] == 0 else f"{s['hz']:5.1f}Hz"
        print(f"     {_pad(c['name'], 14)} {state:>8}  (기대 {c['hz']:>4}){gap}{low}")

    # 외부센서 (표기)
    print('  외부센서')
    for g, s in r['sensors'].items():
        st = r['sensor_state'][g]
        c = SENSOR_TOPICS[g]
        detail = '데이터 없음' if s['count'] == 0 else f"{s['hz']:.1f}Hz"
        if s['gaps']:
            first = s['gaps'][0]
            detail += f"  공백 {len(s['gaps'])}회 (첫 공백 {first[0]:.0f}초부터 {first[1]:.0f}초)"
        print(f"     {MARK[st]} {g:10} {_pad(st, 5)} {detail}")

    # 부가 정보
    m = r['meta']
    if m:
        ev = m.get('sensor_events') or []
        pf = '정상' if m.get('preflight_ok') else f"경고 — {m.get('preflight', '')}"
        print(f"  프리플라이트  {pf}")
        if ev:
            print(f"  비행 중 이벤트 {len(ev)}건: " +
                  ', '.join(f"{e['t_s']:.0f}초 {e['sensor']} {e['from']}→{e['to']}" for e in ev[:4])
                  + (' …' if len(ev) > 4 else ''))
    if r['unclosed']:
        print('  마감 안 됨    metadata.yaml 없음 — 전원 차단으로 종료된 것으로 보입니다')
        print(f"                복구: ros2 bag reindex {r['bag']}")
    if r['log_issues']:
        print(f"  녹화 로그     경고/오류 {r['log_issues']}줄 → {r['name']}_record.log 확인")

    print('─' * 64)
    verdict = r['drone_level']
    note = ' — ' + ', '.join(r['drone_reasons'][:3]) if r['drone_reasons'] else ''
    lacking = [g for g, st in r['sensor_state'].items() if st != '정상']
    sens = f"   외부센서: {', '.join(lacking)} 확인 필요" if lacking else '   외부센서: 전부 정상'
    print(f"판정: 드론 {verdict}{note}")
    print(sens)


def print_table(results):
    print(f"\n{'bag':34} {'길이':>5}  {_pad('드론', 5)} {_pad('MIC', 5)} {_pad('THL', 5)} {_pad('WCM', 5)} 비고")
    print('─' * 86)
    for r in results:
        s = r['sensor_state']
        note = ', '.join(r['drone_reasons'][:2])
        print(f"{r['name'][:34]:34} {r['duration']:5.0f}s  {_pad(r['drone_level'], 5)} "
              f"{_pad(s['ReSpeaker'], 5)} {_pad(s['THL100'], 5)} {_pad(s['WCM6800'], 5)} {note[:30]}")
    print('─' * 86)
    cnt = lambda lv: sum(r['drone_level'] == lv for r in results)
    full = sum(r['drone_level'] == '정상' and all(v == '정상' for v in r['sensor_state'].values())
               for r in results)
    print(f"드론 정상 {cnt('정상')} / 주의 {cnt('주의')} / 불량 {cnt('불량')}   "
          f"(드론·외부센서 모두 정상 {full}개)")


def main():
    # head/less 로 넘길 때 파이프가 닫혀도 오류 없이 종료.
    # 모듈을 import 할 때 등록하면 다른 스레드에서 불러올 수 없으므로 명령줄 실행에서만 합니다.
    signal.signal(signal.SIGPIPE, signal.SIG_DFL)
    args = sys.argv[1:]
    if '-h' in args or '--help' in args:
        print(__doc__)
        return

    csv_out = args[args.index('--csv') + 1] if '--csv' in args else None
    if '--all' in args:
        bags = list_bags()
        if not bags:
            print('bag 이 없습니다:', data_dir()); sys.exit(1)
        print(f'검증 대상 {len(bags)}개 ({data_dir()})')
        results = [verify(b, with_arm=False) for b in bags]
        print_table(results)
        if csv_out:
            import csv
            with open(csv_out, 'w', newline='', encoding='utf-8-sig') as f:
                w = csv.writer(f)
                w.writerow(['bag', 'duration_s', 'drone', 'reasons', 'ReSpeaker', 'THL100', 'WCM6800'])
                for r in results:
                    s = r['sensor_state']
                    w.writerow([r['name'], round(r['duration'], 1), r['drone_level'],
                                '; '.join(r['drone_reasons']), s['ReSpeaker'], s['THL100'], s['WCM6800']])
            print('CSV 저장:', csv_out)
        return

    names = [a for a in args if not a.startswith('--') and a != csv_out]
    if names:
        target = names[0]
        if not os.path.isdir(target):
            target = os.path.join(data_dir(), target)
        if not os.path.isdir(target):
            print('bag 을 찾을 수 없습니다:', names[0]); sys.exit(1)
    else:
        bags = list_bags()
        if not bags:
            print('bag 이 없습니다:', data_dir()); sys.exit(1)
        target = bags[-1]
    r = verify(target)
    print_one(r)
    sys.exit(0 if r['drone_level'] != '불량' else 2)


if __name__ == '__main__':
    main()
