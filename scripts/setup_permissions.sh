#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# 권한 일괄 설정 (install.sh, setup_onboard_env.sh 공통)
#
# 새 모듈에 설치할 때 반복되던 권한 문제를 한 번에 처리합니다.
#   1. udev 규칙  — USB 시리얼·ReSpeaker 를 재로그인 없이 바로 쓸 수 있게
#   2. 그룹       — dialout(시리얼), audio(마이크)
#   3. 소유권     — 예전에 sudo 로 실행해 root 소유가 된 파일 정리
#   4. 점검       — 실제로 열 수 있는지 확인
#
# 사용법:
#   bash scripts/setup_permissions.sh          # 설정 + 점검
#   bash scripts/setup_permissions.sh check    # 점검만
# ══════════════════════════════════════════════════════════════════════════════

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ANOMALY_WS:-$(dirname "$SCRIPT_DIR")}"
DATA="${ANOMALY_DATA:-$HOME/anomaly_data}"
USER_NAME="$(id -un)"
MODE="${1:-apply}"
PROBLEMS=0

ok()   { echo "  ✅ $*"; }
warn() { echo "  ⚠️  $*"; }
bad()  { echo "  ❌ $*"; PROBLEMS=$((PROBLEMS + 1)); }

if [ "$(id -u)" = "0" ]; then
    echo "❌ root 로 실행하지 마세요. 일반 사용자로 실행하면 필요한 곳에서만 sudo 를 요청합니다."
    echo "   (root 로 실행하면 파일이 root 소유가 되어 이후 계속 권한 문제가 생깁니다)"
    exit 1
fi

# ══════════════════════════════════════════════════════════════════════════════
# 설정
# ══════════════════════════════════════════════════════════════════════════════
if [ "$MODE" != "check" ]; then

    # sudo 를 쓸 수 없으면 이후 단계가 모두 실패하므로 먼저 확인합니다
    if ! command -v sudo > /dev/null 2>&1 || ! sudo -v; then
        bad "sudo 를 사용할 수 없어 권한을 설정하지 못했습니다 (관리자 권한 필요)"
        MODE=check
    fi
fi

if [ "$MODE" != "check" ]; then

    # ── 1. udev 규칙 ─────────────────────────────────────────────────────
    # 그룹(dialout) 만으로는 추가 후 재로그인해야 적용됩니다. 새 모듈에서는
    # 설치 직후 확인 명령이 전부 Permission denied 로 실패하므로,
    # USB 시리얼 장치 자체에 읽기·쓰기 권한을 줍니다.
    # (이 모듈은 데이터 수집 전용 장비이므로 장치 권한을 여는 쪽을 택했습니다)
    sudo tee /etc/udev/rules.d/60-respeaker.rules > /dev/null << 'EOF'
# ReSpeaker Mic Array v3.0 — 설정 제어(USB) 와 오디오 장치
SUBSYSTEM=="usb", ATTR{idVendor}=="2886", MODE="0666"
EOF
    sudo tee /etc/udev/rules.d/85-anomaly-serial.rules > /dev/null << 'EOF'
# anomaly_sensor_ros2 — USB 시리얼 장치 권한
# FC·THL100·WCM6800 젠더가 어떤 칩이든, 재로그인 없이 바로 열 수 있게 합니다.
SUBSYSTEM=="tty", KERNEL=="ttyUSB[0-9]*", MODE="0666"
SUBSYSTEM=="tty", KERNEL=="ttyACM[0-9]*", MODE="0666"

# CH340 — brltty 가 점자 장치로 오인해 가로채지 않도록 제외
ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ENV{BRLTTY_BRAILLE_DRIVER}="", ENV{BRLTTY_NO_DRIVER}="1"
EOF
    sudo udevadm control --reload-rules
    # 이미 꽂혀 있는 장치에도 새 권한을 바로 적용합니다
    sudo udevadm trigger --subsystem-match=tty --action=change
    sudo udevadm trigger --subsystem-match=usb --attr-match=idVendor=2886 --action=change 2>/dev/null || true
    sudo udevadm settle --timeout=10 2>/dev/null || true
    if grep -q 'ttyUSB\[0-9\]\*", MODE="0666"' /etc/udev/rules.d/85-anomaly-serial.rules 2>/dev/null; then
        ok "udev 규칙 적용 (USB 시리얼·ReSpeaker)"
    else
        bad "udev 규칙 파일을 쓰지 못했습니다"
    fi

    # ── 2. 그룹 ──────────────────────────────────────────────────────────
    # udev 규칙으로 충분하지만, 규칙이 없는 다른 장치·서비스를 위해 함께 둡니다.
    for g in dialout audio; do
        if id -nG "$USER_NAME" | tr ' ' '\n' | grep -qx "$g"; then
            ok "$g 그룹 등록됨"
        else
            sudo usermod -aG "$g" "$USER_NAME" && ok "$g 그룹 추가" || bad "$g 그룹 추가 실패"
        fi
    done

    # ── 3. 소유권 ────────────────────────────────────────────────────────
    # 예전에 sudo 로 install.sh 를 돌렸거나 서비스가 root 로 파일을 만들었으면
    # 빌드·녹화·로그 기록이 실패합니다.
    mkdir -p "$DATA" 2>/dev/null || sudo mkdir -p "$DATA" || bad "$DATA 생성 실패"
    for d in "$WS" "$DATA"; do
        if [ -d "$d" ] && [ -n "$(find "$d" -xdev ! -user "$USER_NAME" -print -quit 2>/dev/null)" ]; then
            if sudo chown -R "$USER_NAME:$(id -gn)" "$d"; then
                ok "소유권 정리: $d"
            else
                bad "소유권 정리 실패: $d"
            fi
        fi
    done
fi

# ══════════════════════════════════════════════════════════════════════════════
# 점검 — 실제로 열 수 있는지
# ══════════════════════════════════════════════════════════════════════════════
echo ""
echo "  [권한 점검 — 사용자 $USER_NAME]"

ports=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null)
if [ -z "$ports" ]; then
    warn "USB 시리얼 장치 없음 — 젠더를 연결하면 자동으로 권한이 적용됩니다"
else
    for p in $ports; do
        if [ -r "$p" ] && [ -w "$p" ]; then
            ok "$p 읽기·쓰기 가능"
        else
            bad "$p 열 수 없음 ($(stat -c '%A %G' "$p")) — USB 를 뽑았다 다시 꽂아 보세요"
        fi
    done
fi

if lsusb 2>/dev/null | grep -q "2886:"; then
    dev=$(lsusb | awk '/2886:/{printf "/dev/bus/usb/%s/%s", $2, substr($4,1,3); exit}')
    if [ -n "$dev" ] && [ -r "$dev" ] && [ -w "$dev" ]; then
        ok "ReSpeaker USB 제어 가능"
    else
        bad "ReSpeaker USB 제어 불가 ($dev) — 뽑았다 다시 꽂아 보세요"
    fi
fi

if ls /dev/snd/pcmC*c >/dev/null 2>&1; then
    if [ -r "$(ls /dev/snd/pcmC*c | head -1)" ]; then
        ok "오디오 입력 장치 접근 가능"
    else
        warn "오디오 장치 접근 불가 — audio 그룹 적용에 재로그인 필요 (서비스는 영향 없음)"
    fi
fi

for d in "$WS" "$DATA"; do
    if [ ! -w "$d" ]; then
        bad "$d 쓰기 불가"
    elif [ -n "$(find "$d" -xdev ! -user "$USER_NAME" -print -quit 2>/dev/null)" ]; then
        bad "$d 에 다른 사용자 소유 파일이 남아 있음 (예전 sudo 실행 흔적)"
    else
        ok "$d 쓰기 가능"
    fi
done

echo ""
if [ "$PROBLEMS" -eq 0 ]; then
    echo "  권한 문제 없음 — 재로그인 없이 바로 사용할 수 있습니다"
else
    echo "  권한 문제 ${PROBLEMS}건 — 위 안내를 따르세요"
fi
exit 0
