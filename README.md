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
| `drone_sensors` | 통합 launch 패키지 + 자동 녹화 노드(`auto_record_node`) |

```
anomaly_sensor_ros2/
├── src/
│   ├── respeaker/
│   ├── thl100_sensor/
│   ├── wcm6800_sensor/
│   └── drone_sensors/
│       └── launch/drone_sensor_launch.py
├── scripts/
│   ├── record_data.sh         # rosbag 수동 녹화
│   ├── analyze_bag.py         # CSV 변환 + 10Hz 정렬 + 그래프
│   ├── check_time_sync.sh     # 시간 동기화 확인
│   ├── start_onboard.sh       # 온보드 자동 실행 (systemd 호출)
│   ├── install_service.sh     # 부팅 자동 실행 서비스 등록
│   ├── check_record.sh        # 자동 녹화 상태 점검
│   ├── guard_service.sh       # 수동 실행 시 서비스 충돌 방지
│   ├── watch_fcu.sh           # FC 연결 감시 / 자동 복구
│   ├── setup_onboard_env.sh   # 온보드 환경 일괄 설정
│   └── monitor_drone.sh       # 실시간 모니터 (arm/녹화 상태)
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
| `check_record` | `scripts/check_record.sh` | 자동 녹화/서비스 상태 점검 |
| `onboard_log` | `tail -f ~/anomaly_data/onboard.log` | 온보드 실행 로그 확인 |
| `service_status` | `install_service.sh status` | 부팅 자동실행 모드 확인 |
| `watch_fcu` | `watch_fcu.sh` | FC 연결 감시 / 상태 점검 |
| `onboard_env` | `setup_onboard_env.sh` | 온보드 환경 설정 / 상태 확인 |
| `monitor_drone` | `monitor_drone.sh` | 실시간 모니터 (arm/녹화 전환 추적) |

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

## FC 연결 설정

### 연결 방식

기본값은 **TELEM2 + USB-TTL 젠더(CH340)** 구성입니다.

| 연결 | 장치 예시 | baud | SR 파라미터 |
|---|---|---|---|
| **TELEM2** (기본) | `usb-1a86_USB_Serial-if00-port0` | 921600 | `SR2_*` |
| TELEM1 | 동일 젠더 | 57600 | `SR1_*` |
| USB 직결 | `usb-Hex_ProfiCNC_CubeOrange_...-if00` | 115200 | `SR0_*` |

**포트가 바뀌면 SR 파라미터 접두어도 바뀝니다.** USB로 `SR0_*`를 설정해두고 TELEM2로 옮기면 데이터가 오지 않으니 `SR2_*`를 다시 설정해야 합니다.

```bash
check_usb                       # 연결된 장치 확인

ros2 launch drone_sensors drone_sensor_launch.py \
  fcu_url:=/dev/ttyACM0:115200  # 다른 포트/속도로 실행
```

기본값을 바꾸려면 `src/drone_sensors/launch/drone_sensor_launch.py` 상단의 `DEFAULT_FCU_URL`을 수정하세요.

### FC 측 시리얼 설정

Mission Planner 또는 QGroundControl에서 설정합니다.

| 파라미터 | 값 | 비고 |
|---|---|---|
| `SERIAL2_PROTOCOL` | 2 | MAVLink2. `-1`이면 포트 비활성 |
| `SERIAL2_BAUD` | 921 | 921600. 115=115200, 57=57600 |
| `SERIAL2_OPTIONS` | 0 | 흐름제어 OFF (젠더에 CTS/RTS 없을 때 필수) |

### 권장 SR 파라미터 (이상탐지용)

ArduPilot은 기본적으로 일부 메시지만 전송합니다. 아래 값을 **Mission Planner의 Full Parameter List에서 직접 설정**하세요. FC에 영구 저장되므로 포트당 1회만 하면 됩니다.

| 파라미터 | 권장값 | 담당 토픽 | 데이터 |
|---|---|---|---|
| `SR2_RAW_SENS` | **50** | `imu/data_raw`, `imu/mag` | 가속도, Gyro, 지자기 |
| `SR2_EXTRA1` | **50** | `imu/data`, `setpoint_raw/target_attitude` | 자세·각속도, 목표 자세·각속도 |
| `SR2_RAW_CTRL` | **50** | `rc/out` | 모터 PWM 1~8 |
| `SR2_RC_CHAN` | **50** | `rc/in`, `rc/out` | RC 입력, 모터 PWM |
| `SR2_POSITION` | **30** | `local_position/*`, `global_position/*`, `target_local` | 위치·속도, 목표 위치·속도 |
| `SR2_EXTRA3` | **20** | `vibration/*`, `esc_telemetry/*`, `wind_estimation` | 진동, ESC |
| `SR2_EXT_STAT` | **10** | `battery`, `sys_status`, `extended_state`, GPS | 배터리, GPS |
| `SR2_EXTRA2` | **10** | `vfr_hud` | 속도·고도 요약 |

USB 직결이면 `SR0_*`, TELEM1이면 `SR1_*`로 동일하게 설정합니다.

**대역폭 검토** — 921600bps는 초당 약 92KB를 전송할 수 있고, 위 설정의 예상 트래픽은 15~20KB/s로 약 25% 수준입니다. 여유가 충분합니다.

**SR 방식을 쓰는 이유** — ArduPilot은 정밀 제어 시 `MAV_CMD_SET_MESSAGE_INTERVAL`(메시지별 개별 요청)을 권장하지만, 이 방식은 **휘발성이라 FC 재부팅 시 소실**됩니다. 무인 운용에서는 매 부팅마다 재요청해야 하고 한 번 실패하면 해당 비행 데이터가 비어버립니다. SR 파라미터는 FC에 영구 저장되어 그런 위험이 없고, 921600bps에서는 그룹 방식의 대역폭 비효율도 문제가 되지 않습니다.

### 설정 후 실측 확인

SR 값은 요청일 뿐 보장이 아닙니다. FC 부하나 링크 품질에 따라 실제 수신 주기가 낮아질 수 있으므로 확인이 필요합니다.

```bash
for t in /mavros/imu/data /mavros/imu/data_raw /mavros/rc/out \
         /mavros/local_position/pose /mavros/vibration/raw/vibration \
         /mavros/battery; do
  echo -n "$(printf '%-42s' $t): "
  timeout 5 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate" || echo "없음"
done
```

일부 토픽은 조건부로만 발행됩니다.

| 토픽 | 발행 조건 |
|---|---|
| `local_position/*` | GPS fix로 EKF 수렴 필요 |
| `setpoint_raw/target_*` | arm 상태에서 제어 루프 동작 시 |
| `esc_telemetry/*` | ESC 텔레메트리 지원 ESC 필요 |
| `gpsstatus/gps1/raw` | GPS 연결 필요 |
| `battery2` | 배터리 모니터 2번 설정 시 |

### baud rate 확인

연결이 안 될 때 어떤 속도에서 MAVLink가 오는지 확인하는 방법입니다.

```bash
python3 -c "
import serial, time
PORT='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
for b in (921600, 460800, 115200, 57600, 38400, 19200, 9600):
    try:
        s = serial.Serial(PORT, b, timeout=2)
        s.reset_input_buffer(); time.sleep(1.5)
        d = s.read(s.in_waiting or 1); s.close()
        print(f'{b:>7}: {len(d):5d} bytes  fd={d.count(bytes([0xfd])):3d}  {d[:12].hex()}')
    except Exception as e:
        print(f'{b:>7}: ERROR {e}')
"
```

`fd`(MAVLink2 시작 바이트)가 여러 개 나오는 속도가 정답입니다. 어떤 속도에서도 안 나오면 배선(TX/RX 크로스, GND 공통)과 `SERIAL2_PROTOCOL` 설정을 확인하세요.

### 배선 (CubeOrange TELEM2)

```
1: VCC(5V)   2: TX   3: RX   4: CTS   5: RTS   6: GND
```

| FC | 젠더 | 비고 |
|---|---|---|
| 2 (TX) | RX | 크로스 필수 |
| 3 (RX) | TX | 크로스 필수 |
| 6 (GND) | GND | 필수 |
| 1 (VCC) | — | **연결 금지** (젠더는 USB에서 급전) |

TELEM 포트는 **3.3V 로직**입니다. 젠더에 3.3V/5V 점퍼가 있으면 3.3V로 설정하세요.

---


