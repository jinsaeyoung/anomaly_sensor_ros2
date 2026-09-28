#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# 최신 코드 반영 (update_drone)
#
#   git pull → 패키징 → 빌드 → 테스트 → (실행 중이었으면) 서비스 재시작
#
# 파일을 하나씩 복사하면 빠뜨리기 쉽습니다. 이 스크립트는 저장소 전체를
# 한 번에 맞추고, 테스트가 실패하면 서비스를 재시작하지 않고 멈춥니다.
#
# 사용법:
#   update_drone              # GitHub 최신 반영
#   update_drone --no-pull    # 로컬 수정 후 다시 빌드만
# ══════════════════════════════════════════════════════════════════════════════

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ANOMALY_WS:-$(dirname "$SCRIPT_DIR")}"
SERVICE="anomaly-sensor"
PULL=1
[ "${1:-}" = "--no-pull" ] && PULL=0

say()  { echo "[update] $*"; }
fail() { echo "[update] ❌ $*"; exit 1; }

cd "$WS" || fail "워크스페이스 없음: $WS"
set +u; source /opt/ros/humble/setup.bash; set -u

before=$(git rev-parse --short HEAD 2>/dev/null || echo "?")

# ── 1. 코드 받기 ──────────────────────────────────────────────────────
if [ "$PULL" = "1" ]; then
    if [ -n "$(git status --porcelain --untracked-files=no 2>/dev/null)" ]; then
        echo ""
        git status --short --untracked-files=no
        fail "로컬 수정이 있어 받지 않았습니다. 보관하려면: git stash  /  버리려면: git checkout -- ."
    fi
    say "GitHub 에서 받는 중..."
    git pull --ff-only -q || fail "git pull 실패 (네트워크 또는 이력 충돌)"
fi
after=$(git rev-parse --short HEAD 2>/dev/null || echo "?")

if [ "$before" = "$after" ] && [ "$PULL" = "1" ]; then
    say "이미 최신입니다 ($after) — 빌드는 다시 수행합니다"
else
    say "코드: $before → $after"
    [ "$PULL" = "1" ] && git diff --stat "$before" "$after" 2>/dev/null | tail -n 20 | sed 's/^/         /'
fi

# ── 2. 패키징 + 빌드 ─────────────────────────────────────────────────
say "패키징..."
bash "$WS/fix_packaging.sh" > /tmp/update_packaging.log 2>&1 \
    || fail "fix_packaging 실패 — /tmp/update_packaging.log 확인"

say "빌드..."
colcon build --symlink-install > /tmp/update_build.log 2>&1 \
    || { tail -n 20 /tmp/update_build.log; fail "빌드 실패 — /tmp/update_build.log 확인"; }
set +u; source "$WS/install/setup.bash"; set -u

# ── 3. 테스트 — 실패하면 서비스를 건드리지 않습니다 ──────────────────
say "테스트..."
if ! python3 "$WS/tests/test_parsers.py" > /tmp/update_test.log 2>&1; then
    grep -E "^(FAIL|ERROR):" /tmp/update_test.log | head -n 10
    fail "테스트 실패 — 서비스는 재시작하지 않았습니다 (/tmp/update_test.log)"
fi
say "테스트 $(grep -oE 'Ran [0-9]+ tests' /tmp/update_test.log) — 통과"

# ── 4. 서비스 재시작 (실행 중이었을 때만) ────────────────────────────
if systemctl is-active --quiet "$SERVICE" 2>/dev/null; then
    say "서비스 재시작..."
    sudo systemctl restart "$SERVICE" || fail "서비스 재시작 실패"
    say "재시작 완료 — 약 40초 뒤 monitor_drone 으로 확인하세요"
else
    say "서비스가 실행 중이 아니어서 재시작하지 않았습니다"
fi

# alias 목록이 바뀌었으면 install.sh 를 다시 실행해야 합니다
if [ "$PULL" = "1" ] && git diff --name-only "$before" "$after" 2>/dev/null | grep -q '^install.sh$'; then
    say "⚠ install.sh 가 바뀌었습니다 — 새 명령을 쓰려면: bash install.sh && source ~/.bashrc"
fi

say "✅ 완료 ($after)"
