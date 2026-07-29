# anomaly_sensor_ros2

드론 이상탐지(Anomaly Detection)를 위한 다중 센서 데이터 수집 ROS2 워크스페이스.

MAVROS 원본 토픽을 변환 없이 그대로 녹화하고, UART 센서(온습도/전류)는 수신과 발행을 분리해 지정 주기로 발행합니다. 분석 스크립트가 원본 타임스탬프 기준으로 10Hz 정렬 CSV와 그래프를 생성합니다.

---

## 시스템 구조

```
FC (ArduPilot)
    ↓ MAVLink (USB/UART)
mavros_node  →  /mavros/*  (원본 header.stamp 보존) ────┐
                                                         │
ReSpeaker Mic Array v3.0  →  /respeaker/*               │
                                                         ├→ rosbag 녹화
THL100 (UART 수신 스레드)  →  /thl100/data, /raw        │
                                                         │
WCM6800 (UART 수신 스레드) →  /wcm6800/data, /raw       ┘
                                                         ↓
                                              analyze_drone
                                                         ↓
                              10Hz 정렬 merged CSV (age_ms/stale 포함) + 그래프
```

**설계 원칙**
- MAVROS 원본 토픽을 재발행하지 않고 그대로 녹화 — 변환 오류 제거, 사후 재분석 가능
- UART 센서는 수신 스레드와 발행 타이머 분리 — 안정적인 주기 보장
- 좌표계·단위 변환은 수집 단계가 아닌 **분석 단계**에서 수행
- 시간 기준은 **원본 발생 시각**(`header.stamp` / UART 수신 시각) 우선

---

## 패키지 구성

| 패키지 | 설명 |
|---|---|
| `respeaker` | ReSpeaker Mic Array v3.0 DoA/VAD/Audio/Energy |
| `thl100_sensor` | OSTSen-THL100 온습도/조도 (UART 수신 + 1Hz 발행) |
| `wcm6800_sensor` | Winson WCM6800 전류계 (UART 수신 + 10Hz 발행) |
| `drone_sensors` | 통합 launch 패키지 |

```
anomaly_sensor_ros2/
├── src/
│   ├── respeaker/
│   ├── thl100_sensor/
│   ├── wcm6800_sensor/
│   └── drone_sensors/
│       └── launch/drone_sensor_launch.py
├── scripts/
│   ├── record_data.sh         # rosbag 녹화
│   ├── analyze_bag.py         # CSV 변환 + 10Hz 정렬 + 그래프
│   └── check_time_sync.sh     # 시간 동기화 확인
├── tests/
│   ├── test_parsers.py        # 파서/좌표변환 단위 테스트
│   └── virtual_uart_test.sh   # 가상 UART 통합 테스트
├── fix_packaging.sh           # ROS2 패키지 구조 표준화
├── install.sh                 # 전체 환경 자동 설치
├── .gitignore
└── README.md
```

---

## 동작 확인 환경

- Ubuntu 22.04 + ROS2 Humble (x86_64)
- Raspberry Pi 4 + Ubuntu Server 22.04 + ROS2 Humble (arm64)

## 하드웨어

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

install.sh가 자동 처리하는 것:
- ROS2 환경 확인
- 시스템 의존성 설치 (mavros, diagnostic-updater, pyaudio, colcon 등)
- pip 업그레이드 및 `~/.local/bin` PATH 등록
- Python 의존성 설치 (pyusb, pyserial, numpy<2, pandas, matplotlib)
- GeographicLib 데이터 설치
- udev 규칙 설정 (ReSpeaker, dialout 그룹)
- **ROS2 패키지 구조 표준화** (resource 마커, setup.py data_files, setup.cfg underscore 키)
- 이전 빌드 산출물 정리 후 빌드
- 예전 워크스페이스 `.bashrc` 잔재 자동 제거
- 편의 alias 등록

설치 후 **로그아웃 → 재로그인** 필요 (dialout 그룹 권한 적용).

---

## 편의 alias

`.bashrc`에 자동 등록됩니다. 새 터미널을 열거나 `source ~/.bashrc` 후 사용 가능합니다.

| alias | 동작 | 설명 |
|---|---|---|
| `start_drone` | 시간 동기화 확인 → mavros 정리 → launch | 전체 센서 실행 |
| `stop_drone` | mavros/launch 프로세스 종료 | 전체 종료 |
| `check_topics` | 관련 토픽 필터링 출력 | 발행 중인 토픽 확인 |
| `check_usb` | `ls -la /dev/serial/by-id/` | USB 시리얼 장치 확인 |
| `record_drone` | `scripts/record_data.sh` | rosbag 녹화 |
| `analyze_drone` | `python3 scripts/analyze_bag.py` | CSV + 그래프 생성 |

```bash
start_drone                                          # 전체 실행
record_drone 30                                      # 30초 녹화
analyze_drone ~/anomaly_data/anomaly_data_20260616_160131
check_usb
stop_drone
```

---

## 장치 경로 설정

```bash
check_usb
```

기본값은 `src/drone_sensors/launch/drone_sensor_launch.py` 상단 상수로 정의되어 있으며, **재빌드 없이 launch 인자로 덮어쓸 수 있습니다.**

```bash
ros2 launch drone_sensors drone_sensor_launch.py \
  fcu_url:=/dev/ttyACM0:115200 \
  thl100_port:=/dev/ttyUSB0 \
  wcm6800_port:=/dev/ttyUSB1
```

| 인자 | 기본값 | 설명 |
|---|---|---|
| `fcu_url` | CubeOrange by-id:115200 | FC 연결 (USB 115200 / TELEM2 57600) |
| `thl100_port` | Prolific by-id | THL100 시리얼 포트 |
| `wcm6800_port` | CP2102N by-id | WCM6800 시리얼 포트 |
| `thl100_rate` | 1.0 | THL100 발행 Hz |
| `wcm6800_rate` | 10.0 | WCM6800 발행 Hz |
| `respeaker_update_rate` | 50.0 | ReSpeaker DoA/VAD 폴링 Hz |

---

## FC 데이터 스트림 설정 (최초 1회)

ArduPilot은 기본적으로 일부 메시지만 전송합니다. FC에 영구 저장되므로 1회만 설정하면 됩니다.

```bash
for p in SR0_RAW_SENS SR0_EXT_STAT SR0_RC_CHAN SR0_POSITION SR0_EXTRA1 SR0_EXTRA2 SR0_EXTRA3; do
  ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 \
    "{param_id: '$p', value: {integer_value: 10}}"
done
```

진동(VIBE) 데이터는 `SR0_EXTRA3` 설정이 필요합니다.

---

## UART 센서 발행 구조

```
UART 수신 스레드 (연속 수신, 자동 재연결)
    ↓ 스트림 버퍼 파서 → 완전한 패킷만 추출
latest_data + 수신 시각(ROS clock + monotonic) 저장
    ↓
ROS Timer (publish_rate_hz)
    ↓ stale_timeout 이내 데이터만
토픽 발행
```

| 센서 | 발행 주기 | stale timeout | 근거 |
|---|---|---|---|
| THL100 | 1Hz | 5초 | 환경값은 급격히 변하지 않음 |
| WCM6800 | 10Hz | 2초 | 모터·배터리 이상 변화 관찰 |

### 노드 파라미터

| 파라미터 | THL100 | WCM6800 | 설명 |
|---|---|---|---|
| `publish_rate_hz` | 1.0 | 10.0 | 발행 주기 |
| `stale_timeout_sec` | 5.0 | 2.0 | 이 시간 초과 시 발행 보류 |
| `reconnect_delay_sec` | 2.0 | 2.0 | 재연결 재시도 간격 |
| `drain_max_sec` | 3.0 | 3.0 | 연결 직후 버퍼 폐기 최대 시간 |
| `skip_duplicate` | false | — | 동일 sequence 중복 발행 방지 |

### 주요 특징

**연결 직후 누적 버퍼 폐기 (drain)**

PC 부팅 후 아무도 포트를 읽지 않은 동안 tty/드라이버 버퍼에 과거 패킷이 대량 누적됩니다. 이를 그대로 읽으면 수 시간 전 데이터가 rosbag에 기록되므로, 연결 직후 잔여 데이터를 폐기합니다.

```
종료 조건: 0.15초간 새 데이터 없음 (밀린 것 모두 비움)
          또는 drain_max_sec 초과 (무한 대기 방지)
```

시작 로그에 `(초기 버퍼 NNNNN bytes 폐기)`로 표시됩니다.

**시간 API 분리**

| 용도 | API | 이유 |
|---|---|---|
| 발행 타임스탬프 | `self.get_clock().now()` | ROS 시간 체계 일관성, `use_sim_time` 대응 |
| 경과시간 측정 (stale, drain) | `time.monotonic()` | NTP 시각 점프에 영향받지 않음 |

**기타**
- USB 분리 시 자동 재연결
- 스트림 버퍼 파서 — 나뉘어 온 패킷, 붙어 온 패킷, 쓰레기 데이터 모두 처리
- 30초마다 구간 기준 진단 로그

```
THL100 진단 [30s] rx=28 (0.93Hz) ok=28 fail=0 seq_gap=0 | 누적 rx=419 reconnect=1
WCM6800 진단 [30s] rx=92 (3.07Hz) ok=92 fail=0 | 누적 rx=1387 reconnect=1
```

### 발행 토픽

- `/thl100/data`, `/wcm6800/data` — 지정 주기, JSON (수신 시각 `stamp_sec`/`stamp_nsec`, `age_ms` 포함)
- `/thl100/raw`, `/wcm6800/raw` — 수신 즉시 원본 패킷
- 개별 Float32 토픽 병행 발행 (하위 호환)

---

## 녹화 토픽 (21개)

```
MAVROS   /mavros/state, /imu/data, /imu/data_raw, /imu/mag
         /local_position/pose, /local_position/velocity_local
         /global_position/raw/fix, /global_position/raw/gps_vel
         /vfr_hud, /battery, /rc/out
         /vibration/raw/vibration
         /setpoint_raw/target_attitude, /setpoint_raw/target_local

THL100   /thl100/data, /thl100/raw
WCM6800  /wcm6800/data, /wcm6800/raw
Mic      /respeaker/doa, /respeaker/vad, /respeaker/energy
```

`record_data.sh`는 스크립트 위치로부터 워크스페이스를 자동 탐지하므로 어느 경로에 클론해도 동작합니다.

```bash
ANOMALY_DATA=/mnt/ssd/flight_logs record_drone 60    # 저장 경로 변경
```

---

## 데이터 녹화

```bash
record_drone 60     # 60초
record_drone         # 무제한 (Ctrl+C)
```

`~/anomaly_data/anomaly_data_YYYYMMDD_HHMMSS/` 에 저장됩니다.

---

## 시간 동기화

`start_drone`은 실행 전 시간 동기화 상태를 확인합니다.
- 인터넷 연결 + NTP 미동기화 → 자동 동기화 시도
- 인터넷 없음 → 경고 출력 후 계속 진행

야외 오프라인 비행 시 **비행 전 한 번 인터넷 연결**로 시간을 맞춰두는 것을 권장합니다.

```bash
timedatectl status
```

---

## 데이터 분석

```bash
analyze_drone ~/anomaly_data/anomaly_data_20260616_160131
```

결과 (`~/anomaly_data/analyzed/<bag이름>/`):
```
<bag이름>_csv/                  # 토픽별 개별 CSV
<bag이름>_merged_10hz.csv       # 10Hz 정렬 통합 테이블
<bag이름>_overview.png          # 핵심 지표 그래프
```

### 시간 기준

분석기는 **원본 발생 시각**을 우선 사용합니다.

| 우선순위 | 출처 |
|---|---|
| 1 | MAVROS `msg.header.stamp` (FC 동기화 시각) |
| 2 | UART JSON 내부 수신 시각 (`stamp_sec`/`stamp_nsec`) |
| 3 | rosbag 기록 시각 (Header 없는 메시지) |

개별 CSV에는 네 값이 모두 보존됩니다.
- `source_time_ns` — 데이터 발생 시각
- `bag_time_ns` — rosbag 기록 시각
- `transport_delay_ms` — 두 값의 차이 (USB/ROS 스케줄링 지연)
- `stamp_source` — 어떤 시각을 사용했는지 (`header` / `uart` / `bag` / `bag(skew)`)

**header.stamp 신뢰성 검증**

GPS fix가 없으면 ArduPilot이 부팅 후 경과시간을 타임스탬프로 사용해, `header.stamp`가 시스템 시각과 수 시간까지 어긋날 수 있습니다. 이를 그대로 쓰면 정렬 기준이 무너져 merged CSV 행 수가 폭증합니다.

분석기는 `header.stamp`와 rosbag 기록 시각의 차이가 `MAX_STAMP_SKEW_SEC`(기본 5초)를 넘으면 해당 값을 버리고 `bag_time`을 사용하며, 실행 시 아래처럼 알려줍니다.

```
[경고] header.stamp 가 rosbag 기록 시각과 5.0초 이상 어긋나 bag_time 으로 대체한 토픽:
  /mavros/global_position/raw/fix   597/597 건
  /mavros/rc/out                    587/587 건
```

`stamp_source` 컬럼에서 메시지별로 어떤 시각이 쓰였는지 확인할 수 있습니다. GPS fix를 확보하면 대부분 `header`로 바뀝니다.

### 좌표계

MAVROS는 FC의 NED 데이터를 **ENU**로 변환해 발행합니다. 분석기는 이를 명확히 구분합니다.

| 컬럼 | 좌표계 |
|---|---|
| `LocalENU_X/Y/Z`, `LocalENU_VX/VY/VZ` | ENU (MAVROS 원본) |
| `LocalNED_N/E/D`, `LocalNED_VN/VE/VD` | NED 변환값 (N=ENU_Y, E=ENU_X, D=−ENU_Z) |
| `GPS_CourseAngle` | 항공 course (북 0°, 시계방향) |

### 10Hz 정렬 방식

- 기준 시간축: 100ms 간격
- `merge_asof(direction='backward')` — 미래 데이터 사용 금지
- 센서 그룹별 `*_age_ms` — 해당 값이 얼마나 오래된 것인지
- `*_stale` 플래그 — 허용 age 초과 여부 (1/0)
- 허용 age 초과 시 값은 `NaN` 처리 (오래된 값이 정상값처럼 유지되는 문제 방지)
- `/raw` 문자열 컬럼은 merged CSV에서 제외

센서별 허용 age:

| 그룹 | 허용 age | 그룹 | 허용 age |
|---|---|---|---|
| IMU/ATT/RATE | 0.5초 | GPS/BAT/VIBE | 2.0초 |
| WCM/RCOU/MIC/Mag | 1.0초 | THL100 | 3.0초 |
| LocalENU/VFR/Des | 1.0초 | State | 5.0초 |

### merged CSV 컬럼

| 그룹 | 컬럼 |
|---|---|
| 시간 | `datetime` (KST), `time_ms`, `time_sec` |
| 자세 | `ATT_Roll/Pitch/Yaw`, `ATT_DesRoll/DesPitch/DesYaw` |
| IMU | `IMU_AccX/Y/Z`, `IMU_GyrX/Y/Z`, `IMU_Acc*_raw` |
| 각속도 | `RATE_R/P/Y`, `RATE_RDes/PDes/YDes` |
| 배터리 | `BAT_Volt`, `BAT_Curr`, `BAT_CurrTot`, `BAT_Percent` |
| 모터 | `RCOU_C1~C8` |
| 진동 | `VIBE_X/Y/Z`, `VIBE_Clip0~2` |
| GPS | `GPS_Lat/Lon/Alt/Status`, `GPS_GroundSpeed/CourseAngle` |
| 위치 | `LocalENU_*`, `LocalNED_*`, `Des_ENU_*` |
| HUD | `VFR_GroundSpeed/Alt/Climb/Heading` |
| FC 상태 | `State_Armed/Mode/Connected` |
| 환경 | `THL100_Temp/Humi/Light/Seq` |
| 전류 | `WCM_Current`, `WCM_Type` |
| 마이크 | `MIC_DoA/VAD/Energy` |
| 품질 | `*_age_ms`, `*_stale` |

타임존은 `scripts/analyze_bag.py` 상단 `LOCAL_TZ`로 변경 가능합니다.

---

## 테스트

```bash
# 파서 / 좌표변환 단위 테스트 (ROS2 없이 실행 가능)
python3 tests/test_parsers.py

# 가상 UART 통합 테스트 (실제 센서 없이 노드 검증)
sudo apt install socat
bash tests/virtual_uart_test.sh thl100
bash tests/virtual_uart_test.sh wcm6800
```

---

## MAVLink 경로 한계

| 항목 | 상태 | 대안 |
|---|---|---|
| CTRL RMS (PID 로그) | MAVLink 미지원 | FC BIN 파일 |
| BAT_Res (내부저항) | mavros 미제공 | FC BIN 파일 |
| POWR (전원 플래그) | 일부 미지원 | FC BIN 파일 |
| VIBE X/Y/Z | 지원 | `SR0_EXTRA3` 설정 후 수집 |

---

## 정상 동작 확인 방법

실행 후 30초 뒤 진단 로그가 아래와 같으면 정상입니다.

```
THL100 진단 [30s] rx=28 (0.93Hz) ok=28 fail=0 seq_gap=0
WCM6800 진단 [30s] rx=92 (3.07Hz) ok=92 fail=0
```

- 발행 Hz가 센서 사양과 일치하는가
- `fail=0`, `seq_gap=0` 인가
- 시작 로그에 `(초기 버퍼 ... bytes 폐기)`가 **즉시** 나타나는가

실내 환경에서 아래 mavros 메시지는 정상입니다.

| 메시지 | 의미 |
|---|---|
| `GP: No GPS fix` | 실내라 GPS 미수신. 야외에서 해소 |
| `TM: Wrong FCU time` | GPS 없어 FC 시각 동기화 불가 |
| `PreArm: Check mag field` | 나침반 보정 필요. 데이터 수집에는 무관 |

---

## 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| mavros 실행 실패 (`libdiagnostic_updater.so` 없음) | diagnostic 패키지 미설치/손상 | `sudo apt install ros-humble-diagnostic-updater ros-humble-diagnostic-msgs` (install.sh 반영됨) |
| ReSpeaker USB Pipe error | USB 인터페이스 충돌 | Tuning 클래스가 SEEED Control 인터페이스만 claim하는지 확인 |
| `KeyError: 'launch'` | `drone_sensors/launch/__init__.py` 존재 | install.sh / fix_packaging.sh가 자동 제거 |
| mavros 크래시 (`invalid allocator`) | mavros 프로세스 중복 실행 | `start_drone` alias가 자동 정리 |
| rqt에서 mavros 토픽 안 보임 | QoS 불일치 (BEST_EFFORT) | `ros2 topic echo <topic> --qos-reliability best_effort` |
| `/mavros/imu/data` 미발행 | SR 파라미터 미설정 | "FC 데이터 스트림 설정" 참고 |
| 진동 토픽 비어있음 | `SR0_EXTRA3` 미설정 | SR 파라미터 설정 |
| 노드 시작 직후 수신 Hz가 수천 Hz | PC 부팅 후 tty 버퍼에 과거 패킷 누적 | 연결 시 drain으로 폐기 (적용됨). 진단이 구간 기준이라 이후 정상 Hz 확인 가능 |
| 센서 "N초간 수신 없음" / "발행 보류" 반복 | drain 로직이 종료되지 않아 수신 루프 미시작 | idle_gap + `drain_max_sec` 상한으로 반드시 종료 (적용됨) |
| USB 포트 번호 변경 | 재연결 시 번호 변동 | `/dev/serial/by-id/` 사용 (적용됨) |
| USB 분리 후 센서 복구 안 됨 | 수신 스레드 종료 | 자동 재연결 구현됨 |
| `colcon build` File exists | 이전 빌드 잔여 파일 | install.sh가 빌드 전 자동 정리 |
| `ros2 run` 패키지 못 찾음 | resource 마커/data_files 누락 | `bash fix_packaging.sh` 후 재빌드 |
| GitHub 클론 후 패키지 비어있음 | git submodule 등록 | `git rm --cached <pkg>` → `rm -rf <pkg>/.git` → `git add <pkg>` |
| matplotlib NumPy 오류 | NumPy 2.x 비호환 | `pip install "numpy<2"` (적용됨) |
| THL100 "필드 수 오류" | 패킷 중복 수신 | 스트림 버퍼 파서로 해결 (적용됨) |
| `datetime` 9시간 차이 | UTC → KST 변환 누락 | `LOCAL_TZ` 기준 변환 (적용됨) |
| `datetime`이 부팅 전 날짜 | NTP 미동기화 | `start_drone`이 자동 확인/보정 |
| 오래된 센서값이 계속 유지됨 | tolerance 없는 forward fill | `*_age_ms`/`*_stale` + NaN 처리 (적용됨) |
| stale 판정이 갑자기 오작동 | NTP 동기화로 시스템 시각 점프 | 경과시간을 `time.monotonic()` 기준으로 측정 (적용됨) |
| `record_drone` 경로 오류 | 다른 경로에 클론 | 스크립트가 워크스페이스 자동 탐지 (적용됨) |
| FC 연결 끊김 (`No such device`) | USB 분리/FC 재부팅 | USB 재연결 후 `stop_drone` → `start_drone` |
| `record_drone` 실행 시 `AMENT_TRACE_SETUP_FILES: 바인딩 해제한 변수` | 스크립트의 `set -u`와 ROS `setup.bash`의 미정의 변수 참조 충돌 | ROS source 구간만 `set +u`로 감싸도록 수정 (적용됨) |
| 분석 시 duration이 수 시간, merged CSV가 수십만 행 | GPS fix 없어 FC `header.stamp`가 부팅 경과시간 기준 → 시스템 시각과 큰 차이 | `MAX_STAMP_SKEW_SEC` 초과 시 `bag_time`으로 자동 대체 (적용됨). 실행 시 경고와 `stamp_source` 컬럼으로 확인 가능 |
