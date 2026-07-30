#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# ROS2 Python 패키지 표준화 스크립트
#
# 각 패키지에 다음을 보장합니다:
#   - resource/<package_name> 마커 파일
#   - setup.py의 data_files (ament index 등록 + package.xml 설치)
#   - setup.cfg의 underscore 키 (script_dir / install_scripts)
#
# 사용법: bash fix_packaging.sh
# ══════════════════════════════════════════════════════════════════════════════

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ANOMALY_WS:-$SCRIPT_DIR}"
SRC="$WS/src"

echo "=========================================="
echo " ROS2 패키지 표준화"
echo " 워크스페이스: $WS"
echo "=========================================="

# 노드 패키지별 entry point 정의 ("|"로 여러 개 구분)
declare -A ENTRY_POINTS=(
  ["respeaker"]="respeaker_node = respeaker.respeaker_node:main|respeaker_full_node = respeaker.respeaker_full_node:main"
  ["thl100_sensor"]="thl100_node = thl100_sensor.thl100_uart_node:main"
  ["wcm6800_sensor"]="wcm6800_node = wcm6800_sensor.wcm6800_uart_node:main"
)

for PKG in respeaker thl100_sensor wcm6800_sensor; do
    PKG_DIR="$SRC/$PKG"
    if [ ! -d "$PKG_DIR" ]; then
        echo "  건너뜀: $PKG (폴더 없음)"
        continue
    fi

    echo ""
    echo "[$PKG] 처리 중..."

    # 1) resource 마커 파일
    mkdir -p "$PKG_DIR/resource"
    touch "$PKG_DIR/resource/$PKG"
    echo "  ✓ resource/$PKG"

    # 2) setup.cfg (underscore 키)
    cat > "$PKG_DIR/setup.cfg" << CFGEOF
[develop]
script_dir=\$base/lib/$PKG
[install]
install_scripts=\$base/lib/$PKG
CFGEOF
    echo "  ✓ setup.cfg"

    # 3) setup.py (data_files 포함)
    IFS='|' read -ra EPS <<< "${ENTRY_POINTS[$PKG]}"
    EP_LINES=""
    for ep in "${EPS[@]}"; do
        EP_LINES="${EP_LINES}            '${ep}',
"
    done

    cat > "$PKG_DIR/setup.py" << PYEOF
from setuptools import setup

package_name = '$PKG'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='$PKG package for anomaly_sensor_ros2',
    license='MIT',
    entry_points={
        'console_scripts': [
${EP_LINES}        ],
    },
)
PYEOF
    echo "  ✓ setup.py"
done

# ── drone_sensors (launch 전용 패키지) ───────────────────────────────
PKG_DIR="$SRC/drone_sensors"
if [ -d "$PKG_DIR" ]; then
    echo ""
    echo "[drone_sensors] 처리 중..."
    mkdir -p "$PKG_DIR/resource" "$PKG_DIR/drone_sensors" "$PKG_DIR/config"
    touch "$PKG_DIR/resource/drone_sensors"
    touch "$PKG_DIR/drone_sensors/__init__.py"

    # launch/__init__.py 제거 (ROS2 launch 모듈 이름 충돌 방지)
    rm -f "$PKG_DIR/launch/__init__.py"

    # auto_record_node 가 rclpy / std_msgs / mavros_msgs 를 사용하므로
    # package.xml 에 의존성이 없으면 자동으로 추가합니다.
    PKG_XML="$PKG_DIR/package.xml"
    if [ -f "$PKG_XML" ]; then
        for dep in rclpy std_msgs mavros_msgs; do
            if ! grep -q "<depend>$dep</depend>" "$PKG_XML"; then
                sed -i "s|</package>|  <depend>$dep</depend>\n</package>|" "$PKG_XML"
                echo "  + package.xml 의존성 추가: $dep"
            fi
        done
    fi

    cat > "$PKG_DIR/setup.cfg" << 'CFGEOF'
[develop]
script_dir=$base/lib/drone_sensors
[install]
install_scripts=$base/lib/drone_sensors
CFGEOF

    cat > "$PKG_DIR/setup.py" << 'PYEOF'
from setuptools import setup
import os
from glob import glob

package_name = 'drone_sensors'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Integrated launch package for anomaly_sensor_ros2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'auto_record_node = drone_sensors.auto_record_node:main',
        ],
    },
)
PYEOF
    echo "  ✓ resource/drone_sensors, setup.py, setup.cfg"
fi

echo ""
echo "=========================================="
echo " 완료 — 재빌드가 필요합니다:"
echo "   cd $WS && rm -rf build install log && colcon build --symlink-install"
echo "=========================================="
