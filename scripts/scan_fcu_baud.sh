#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# FC 시리얼 baud rate 탐색
#
# TELEM 포트나 USB-TTL 젠더로 연결했는데 mavros 가 붙지 않을 때,
# 어떤 통신 속도에서 MAVLink 패킷이 오는지 찾아줍니다.
#
# 사용법:
#   bash scripts/scan_fcu_baud.sh                    # 시리얼 장치 자동 탐색
#   bash scripts/scan_fcu_baud.sh /dev/ttyUSB2       # 포트 직접 지정
# ══════════════════════════════════════════════════════════════════════════════

PORT="${1:-}"

echo "=========================================="
echo " FC baud rate 탐색"
echo "=========================================="

# ── 포트 미지정 시 목록 표시 ──────────────────────────────────────────
if [ -z "$PORT" ]; then
    echo ""
    echo "연결된 시리얼 장치:"
    if ls /dev/serial/by-id/* >/dev/null 2>&1; then
        for f in /dev/serial/by-id/*; do
            echo "  $f"
            echo "    → $(readlink -f "$f")"
        done
    else
        echo "  (없음)"
        exit 1
    fi
    echo ""
    echo "포트를 지정해서 다시 실행하세요:"
    echo "  bash scripts/scan_fcu_baud.sh /dev/serial/by-id/usb-XXXX-if00-port0"
    exit 0
fi

if [ ! -e "$PORT" ]; then
    echo "ERROR: 장치를 찾을 수 없습니다: $PORT"
    exit 1
fi

echo " 대상 포트: $PORT"
echo "=========================================="
echo ""

python3 - "$PORT" << 'PYEOF'
import sys, time
try:
    import serial
except ImportError:
    print("ERROR: pyserial 미설치 →  pip3 install pyserial")
    sys.exit(1)

PORT = sys.argv[1]
BAUDS = (921600, 460800, 115200, 57600, 38400, 19200, 9600)

print(f"{'baud':>8}  {'bytes':>6}  {'MAVLink2':>8}  {'MAVLink1':>8}   head")
print("-" * 62)

best = None
for b in BAUDS:
    try:
        s = serial.Serial(PORT, b, timeout=2)
        s.reset_input_buffer()
        time.sleep(1.5)
        d = s.read(s.in_waiting or 1)
        s.close()
    except Exception as e:
        print(f"{b:>8}  ERROR {e}")
        continue

    mav2 = d.count(b'\xfd')
    mav1 = d.count(b'\xfe')
    head = d[:14].hex()
    mark = ""
    # MAVLink2 프레임 시작 패턴을 우선 판정
    if len(d) > 20 and mav2 >= 2:
        mark = "  <== MAVLink2"
        if best is None:
            best = (b, 'v2')
    elif len(d) > 20 and mav1 >= 2:
        mark = "  <== MAVLink1"
        if best is None:
            best = (b, 'v1')

    print(f"{b:>8}  {len(d):>6}  {mav2:>8}  {mav1:>8}   {head}{mark}")

print()
if best:
    baud, ver = best
    print("=" * 62)
    print(f" 탐지 결과: {baud} bps ({ 'MAVLink2' if ver=='v2' else 'MAVLink1' })")
    print("=" * 62)
    print()
    print(" 이 설정으로 실행하세요:")
    print(f"   ros2 launch drone_sensors drone_sensor_launch.py \\")
    print(f"     fcu_url:={PORT}:{baud}")
    print()
    print(" 기본값으로 고정하려면 launch 파일의 DEFAULT_FCU_URL 을 수정하세요.")
else:
    print("=" * 62)
    print(" MAVLink 패킷을 찾지 못했습니다.")
    print("=" * 62)
    print()
    print(" 확인 사항:")
    print("   1. 배선  FC TX ↔ 젠더 RX,  FC RX ↔ 젠더 TX,  GND 공통")
    print("            (TX/RX 크로스가 가장 흔한 실수 — 뒤집어서 재시도)")
    print("            VCC 는 연결하지 마세요")
    print("   2. FC 파라미터 (USB 로 접속해 Mission Planner 등에서 확인)")
    print("        SERIALn_PROTOCOL = 2   (MAVLink2, -1이면 포트 비활성)")
    print("        SERIALn_BAUD     = 921 / 115 / 57 ...")
    print("        SERIALn_OPTIONS  = 0   (흐름제어 OFF)")
    print("   3. 전압 레벨  TELEM 포트는 3.3V 로직")
    print("                젠더에 3.3V/5V 점퍼가 있으면 3.3V 로 설정")
PYEOF
