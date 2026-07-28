# anomaly_sensor_ros2

드론 이상탐지(Anomaly Detection)를 위한 다중 센서 데이터 수집 ROS2 워크스페이스.

mavros 원본 토픽을 직접 녹화하고, UART 센서(온습도/전류)는 지정 주기로 발행하는 구조로 구성되어 있습니다. 수집된 데이터는 rosbag으로 저장되며, 분석 스크립트가 10Hz 기준 정렬 CSV와 그래프를 자동으로 생성합니다.

---

## 시스템 구조

```
FC (ArduPilot)
    ↓ MAVLink (USB/UART)
mavros_node  →  /mavros/*  ─────────────────────────────┐
                                                         │
ReSpeaker Mic Array v3.0  →  /respeaker/*               │
                                                         ├→ rosbag 녹화
THL100 (UART 연속 수신)   →  /thl100/data, /thl100/raw  │
                                                         │
WCM6800 (UART 연속 수신)  →  /wcm6800/data, /wcm6800/raw┘
                                                         ↓
                                              analyze_drone
                                                         ↓
                                         10Hz 정렬 merged CSV + 그래프
```

**핵심 설계 원칙:**
- mavros 원본 토픽을 변환·재발행하지 않고 그대로 녹화
- UART 센서는 수신 스레드와 발행 타이머를 분리하여 안정적인 주기 발행
- 좌표계 변환, 단위 변환 등은 수집 단계가 아닌 **분석 단계**에서 수행

---

## 패키지 구성

| 패키지 | 설명 |
|---|---|
| `respeaker` | ReSpeaker Mic Array v3.0 DoA/VAD/Audio/Energy |
| `thl100_sensor` | OSTSen-THL100 온습도/조도 (UART 수신 + 1Hz 발행) |
| `wcm6800_sensor` | Winson WCM6800 전류계 (UART 수신 + 10Hz 발행) |
| `drone_sensors` | 전체 통합 launch 패키지 |
| `drone_state` | (비활성) mavros Float32 재발행 노드 — 필요 시 활성화 가능 |

```
anomaly_sensor_ros2/
├── src/
│   ├── respeaker/
│   ├── thl100_sensor/
│   ├── wcm6800_sensor/
│   ├── drone_sensors/
│   │   └── launch/drone_sensor_launch.py
│   └── drone_state/          # 비활성 (launch에서 제외)
├── scripts/
│   ├── record_data.sh         # rosbag 녹화
│   ├── analyze_bag.py         # CSV 변환 + 10Hz 정렬 + 그래프
│   └── check_time_sync.sh     # 시간 동기화 확인
├── install.sh                 # 전체 환경 자동 설치
└── README.md
```

---

## 동작 확인 환경

- Ubuntu 22.04 + ROS2 Humble (x86_64)
- Raspberry Pi 4 + Ubuntu Server 22.04 + ROS2 Humble (arm64)

---

## 하드웨어 요구사항

- ReSpeaker Mic Array v3.0 (USB)
- OSTSen-THL100 온습도/조도계 (UART → USB, 9600bps)
- Winson WCM6800 전류계 (UART → USB, 9600bps)
- ArduPilot FC (CubeOrange 등, USB 또는 TELEM2)

---

## 설치

```bash
git clone https://github.com/jinsaeyoung/anomaly_sensor_ros2.git
cd anomaly_sensor_ros2
bash install.sh
```

설치 스크립트가 자동으로 처리하는 것:
- ROS2 환경 확인
- 시스템 의존성 설치 (mavros, pyaudio, colcon 등)
- pip 업그레이드 및 PATH 설정
- Python 의존성 설치 (pyusb, pyserial, numpy<2, pandas, matplotlib 등)
- GeographicLib 데이터 설치 (mavros 필수)
- udev 규칙 설정 (ReSpeaker, 시리얼 포트 권한)
- 예전 워크스페이스 잔재 `.bashrc` 자동 정리
- ROS2 워크스페이스 빌드 (이전 빌드 산물 정리 후 진행)
- 편의 alias 등록

설치 후 **로그아웃 → 재로그인** 필요 (dialout 그룹 권한 적용).

---

## USB 장치 ID 확인 및 설정

```bash
check_usb   # 또는: ls -la /dev/serial/by-id/
```

확인된 ID를 `src/drone_sensors/launch/drone_sensor_launch.py` 상단에 반영:

```python
# FC 연결 (USB)
FCU_URL = '/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_xxxxxxxx-if00:115200'

# FC 연결 (TELEM2 UART로 변경 시)
# FCU_URL = '/dev/serial/by-id/여기에_TELEM2_장치ID_입력:57600'
```

수정 후 재빌드:
```bash
colcon build --packages-select drone_sensors --symlink-install
```

---

## FC 데이터 스트림 설정 (최초 1회)

ArduPilot은 기본적으로 일부 메시지만 전송합니다. FC에 영구 저장되므로 최초 1회만 설정하면 됩니다.

```bash
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 "{param_id: 'SR0_RAW_SENS', value: {integer_value: 10}}"
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 "{param_id: 'SR0_EXT_STAT', value: {integer_value: 10}}"
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 "{param_id: 'SR0_RC_CHAN',  value: {integer_value: 10}}"
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 "{param_id: 'SR0_POSITION', value: {integer_value: 10}}"
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 "{param_id: 'SR0_EXTRA1',   value: {integer_value: 10}}"
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 "{param_id: 'SR0_EXTRA2',   value: {integer_value: 10}}"
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 "{param_id: 'SR0_EXTRA3',   value: {integer_value: 10}}"
```

---

## 편의 alias (install.sh 자동 등록)

`bash install.sh` 실행 시 `.bashrc`에 자동으로 등록됩니다. 새 터미널을 열거나 `source ~/.bashrc` 후 사용 가능합니다.

| alias | 실제 동작 | 설명 |
|---|---|---|
| `start_drone` | `check_time_sync.sh` → `pkill mavros_node` → `ros2 launch drone_sensors drone_sensor_launch.py` | 시간 동기화 확인 후 이전 mavros 정리, 전체 센서 실행 |
| `stop_drone` | `pkill mavros_node` + `pkill drone_sensor_launch` | 모든 센서 노드 종료 |
| `check_topics` | `ros2 topic list \| grep -E "drone\|mavros\|respeaker\|thl100\|wcm6800"` | 관련 토픽 목록만 필터링해서 출력 |
| `check_usb` | `ls -la /dev/serial/by-id/` | 연결된 USB 시리얼 장치 ID 목록 확인 |
| `record_drone` | `scripts/record_data.sh` | rosbag 녹화 실행 |
| `analyze_drone` | `python3 scripts/analyze_bag.py` | bag 파일 분석 (CSV + 그래프 생성) |

### 사용 예시

```bash
# 전체 센서 실행
start_drone

# 30초 녹화
record_drone 30

# 무제한 녹화 (Ctrl+C로 종료)
record_drone

# 분석
analyze_drone ~/anomaly_data/anomaly_data_20260616_160131

# 연결된 USB 장치 확인
check_usb

# 현재 발행 중인 관련 토픽 확인
check_topics

# 전체 종료
stop_drone
```

> alias가 동작하지 않으면 `source ~/.bashrc` 를 실행하거나 새 터미널을 여세요.

---

## 실행

```bash
start_drone
```

`start_drone`은 다음 순서로 동작합니다:
1. `check_time_sync.sh` — 시간 동기화 확인/보정
2. 이전 mavros 프로세스 정리
3. `ros2 launch drone_sensors drone_sensor_launch.py`

발행 주기를 변경하려면:
```bash
ros2 launch drone_sensors drone_sensor_launch.py \
  thl100_rate:=1.0 \
  wcm6800_rate:=10.0 \
  respeaker_update_rate:=50.0
```

새 터미널에서 토픽 확인:
```bash
check_topics
```

종료:
```bash
stop_drone
```

---

## UART 센서 발행 구조

UART 수신과 토픽 발행을 분리하여 안정적인 주기를 보장합니다.

```
UART 수신 스레드 (계속 수신)
    ↓ 정상 패킷 파싱 시
latest_data 저장
    ↓
ROS Timer (publish_rate_hz 주기)
    ↓ stale_timeout_sec 이내 데이터만
토픽 발행
```

| 센서 | 권장 발행 주기 | stale timeout | 이유 |
|---|---|---|---|
| THL100 온도·습도·조도 | 1Hz | 3초 | 환경값은 급격히 변하지 않음 |
| WCM6800 전류 | 10Hz | 1초 | 모터·배터리 이상 변화 관찰 필요 |

각 센서는 두 종류의 토픽을 발행합니다:
- `/thl100/data`, `/wcm6800/data` — 지정 주기로 최신 정상값 발행 (JSON, Header timestamp 포함)
- `/thl100/raw`, `/wcm6800/raw` — 수신 즉시 원본 패킷 발행

---

## 녹화 토픽 구성

```
MAVROS 원본         /mavros/state, /mavros/imu/data, /mavros/imu/data_raw
                   /mavros/imu/mag, /mavros/local_position/pose
                   /mavros/local_position/velocity_local
                   /mavros/global_position/raw/fix, /mavros/global_position/raw/gps_vel
                   /mavros/vfr_hud, /mavros/battery, /mavros/rc/out
                   /mavros/setpoint_raw/target_attitude, /mavros/setpoint_raw/target_local

THL100             /thl100/data, /thl100/raw
WCM6800            /wcm6800/data, /wcm6800/raw
ReSpeaker          /respeaker/doa, /respeaker/vad, /respeaker/energy
```

---

## 데이터 녹화

```bash
record_drone 60     # 60초 녹화
record_drone         # 무제한 (Ctrl+C로 종료)
```

`~/anomaly_data/anomaly_data_YYYYMMDD_HHMMSS/` 에 저장됩니다.

---

## 시간 동기화 (datetime 정확도)

`start_drone`은 실행 전 자동으로 시간 동기화 상태를 확인합니다:
- 인터넷 연결 + NTP 미동기화 → 자동 동기화 시도
- 인터넷 없음 → 경고만 출력하고 계속 진행

야외 오프라인 비행 시에는 **비행 전 한 번이라도 인터넷에 연결**하여 시간을 맞춰두는 것을 권장합니다.

수동 확인:
```bash
timedatectl status
```

---

## 데이터 분석

```bash
analyze_drone ~/anomaly_data/anomaly_data_20260616_160131
```

결과물 (`~/anomaly_data/analyzed/<bag이름>/`):
```
<bag이름>_csv/                  # 토픽별 개별 CSV
<bag이름>_merged_10hz.csv       # 10Hz 기준 정렬 통합 테이블
<bag이름>_overview.png          # 핵심 지표 그래프
```

`_merged_10hz.csv` 컬럼 구성:

| 그룹 | 컬럼 |
|---|---|
| 시간 | `datetime` (KST), `time_ms`, `time_sec` |
| 자세 | `ATT_Roll/Pitch/Yaw`, `ATT_DesRoll/DesPitch/DesYaw` |
| IMU | `IMU_AccX/Y/Z`, `IMU_GyrX/Y/Z` |
| 각속도 | `RATE_R/P/Y`, `RATE_RDes/PDes/YDes` |
| 배터리 | `BAT_Volt`, `BAT_Curr`, `BAT_CurrTot` |
| 모터 PWM | `RCOU_C1~C8` |
| 진동 | `VIBE_X/Y/Z` |
| GPS | `GPS_Lat/Lon/Alt`, `GPS_GroundSpeed/CourseAngle` |
| 로컬 위치/속도 | `LocalNED_X/Y/Z`, `LocalNED_VX/VY/VZ` |
| FC 상태 | `State_Armed`, `State_Mode`, `State_Connected` |
| 환경 | `THL100_Temp/Humi/Light` |
| 전류 | `WCM_Current`, `WCM_Type` |
| 마이크 | `MIC_DoA`, `MIC_VAD`, `MIC_Energy` |

**10Hz 정렬 방식:**
- 기준 시간축: 100ms 간격
- 모든 값: forward fill (최근값 유지, 미래 데이터 사용 금지)
- 타임존: 기본 Asia/Seoul — `scripts/analyze_bag.py` 상단 `LOCAL_TZ` 변경 가능

---

## 누락 데이터 안내 (MAVLink 경로 한계)

| 항목 | 상태 | 대안 |
|---|---|---|
| CTRL RMS (PID 로그) | MAVLink 미지원 | FC 내부 BIN 파일에서만 취득 가능 |
| BAT_Res (내부저항) | mavros 미제공 | BIN 파일 |
| POWR (전원 플래그) | 일부 미지원 | BIN 파일 |
| VIBE X/Y/Z | `/mavros/vibration/raw/vibration` | SR 파라미터 설정 후 수신 가능 |

---

## 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| ReSpeaker USB Pipe error | USB 인터페이스 충돌 | `respeaker_full_node.py`의 Tuning 클래스가 SEEED Control 인터페이스만 claim하는지 확인 |
| `KeyError: 'launch'` | `drone_sensors/launch/__init__.py` 존재 | `rm src/drone_sensors/launch/__init__.py` 후 재빌드 (install.sh 자동 처리) |
| mavros 크래시 (`invalid allocator`) | 이전 mavros 프로세스 중복 실행 | `pkill -f mavros_node` 후 재실행 (`start_drone` alias가 자동 처리) |
| rqt에서 mavros 토픽 안 보임 | QoS 불일치 (mavros는 BEST_EFFORT) | `ros2 topic echo <topic> --qos-reliability best_effort` |
| `/mavros/imu/data` 등 토픽 미발행 | ArduPilot SR 파라미터 미설정 | 위 "FC 데이터 스트림 설정" 단계 진행 |
| `mavros/vibration` 토픽 비어있음 | SR 파라미터 또는 FC 펌웨어 버전 | `SR0_EXTRA3` 설정 확인 |
| USB 포트 번호 변경 | 재연결 시 번호 변동 | `/dev/serial/by-id/` 사용 (이미 적용됨) |
| `pip install` 옵션 오류 | pip 버전 낮음 | install.sh가 pip 업그레이드 후 재시도 (이미 적용됨) |
| `colcon build` File exists 에러 | 이전 빌드 잔여 파일 충돌 | install.sh가 빌드 전 자동 정리 (이미 적용됨) |
| GitHub 클론 후 패키지 폴더 비어있음 | git submodule로 등록됨 | `git rm --cached <pkg>` → `rm -rf <pkg>/.git` → `git add <pkg>` |
| matplotlib NumPy 오류 | NumPy 2.x 비호환 | `pip install "numpy<2"` (install.sh 적용됨) |
| THL100 "필드 수 오류" | UART 버퍼에 패킷 중복 수신 | `@` 기준 마지막 패킷만 파싱하도록 처리됨 |
| `datetime`이 9시간 빠름 | UTC → KST 변환 누락 | `analyze_bag.py`에서 `LOCAL_TZ` 기준 변환 처리됨 |
| `datetime` 값이 부팅 전 날짜 | NTP 미동기화 | `start_drone` 실행 시 자동 확인/보정 |
| `ros2_ws/install/setup.bash` 없음 | 예전 워크스페이스 잔재 | install.sh가 자동 정리 (이미 적용됨) |
| `record_drone`/`analyze_drone` alias 없음 | `.bashrc` 미적용 | `source ~/.bashrc` 또는 새 터미널 열기 |
| FC 연결 끊김 (`No such device`) | USB 분리 또는 FC 재부팅 | USB 재연결 후 `stop_drone` → `start_drone` |
