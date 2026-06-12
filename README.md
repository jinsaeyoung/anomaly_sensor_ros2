# anomaly_sensor_ros2

드론 이상탐지(Anomaly Detection)를 위한 다중 센서 데이터 수집 ROS2 패키지 모음.

## 구성

| 패키지 | 설명 | 발행 토픽 |
|---|---|---|
| `respeaker` | ReSpeaker Mic Array v3.0 (DoA/VAD/Audio/Energy) | `/respeaker/*` |
| `thl100_sensor` | OSTSen-THL100 온습도/조도 센서 (UART) | `/thl100/*` |
| `wcm6800_sensor` | Winson WCM6800 전류계 (UART) | `/wcm6800/*` |
| `drone_state` | mavros 데이터를 재가공한 커스텀 토픽 (FC 상태) | `/drone/*` |
| `drone_sensors` | 전체 통합 launch 패키지 | - |

## 하드웨어 요구사항

- ReSpeaker Mic Array v3.0 (USB)
- OSTSen-THL100 온습도/조도계 (UART → USB, 9600bps)
- Winson WCM6800 전류계 (UART → USB, 9600bps)
- ArduPilot FC (CubeOrange 등, USB or TELEM)

## 설치

```bash
git clone https://github.com/your-id/anomaly_sensor_ros2.git
cd anomaly_sensor_ros2
bash install.sh
```

설치 스크립트가 자동으로 처리하는 것:
- 시스템/Python 의존성 설치 (mavros, pyaudio, pyserial, pyusb 등)
- GeographicLib 데이터 설치
- udev 규칙 설정 (ReSpeaker, 시리얼 포트 권한)
- ROS2 워크스페이스 빌드
- 편의 alias 등록 (`start_drone`, `stop_drone` 등)

설치 후 **로그아웃 → 재로그인** 필요 (dialout 그룹 권한 적용).

## USB 장치 ID 설정

장치마다 USB 포트 번호(`ttyUSB0`, `ttyACM0` 등)는 재연결 시 바뀔 수 있으므로
`/dev/serial/by-id/` 고유 ID를 사용합니다.

```bash
check_usb   # 또는: ls -la /dev/serial/by-id/
```

확인된 장치 ID를 `drone_sensors/launch/drone_sensor_launch.py`에서 설정:

```python
# FC 연결
FCU_URL = '/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_xxxxxxxx-if00:115200'

# THL100, WCM6800 포트 (DeclareLaunchArgument 기본값)
thl100_port_arg ...  default_value='/dev/serial/by-id/usb-Prolific_xxxx...'
wcm6800_port_arg ... default_value='/dev/serial/by-id/usb-Diwell_xxxx...'
```

## 실행

```bash
start_drone     # mavros + 전체 센서 실행
```

새 터미널에서 토픽 확인:
```bash
check_topics    # 전체 관련 토픽 목록
ros2 topic echo /drone/attitude/roll
ros2 topic echo /thl100/temperature
ros2 topic echo /wcm6800/current
ros2 topic echo /respeaker/doa
```

종료:
```bash
stop_drone
```

## mavros 동작 모드

`drone_sensor_launch.py` 상단에서 설정:

```python
MAVROS_MODE = 'readonly'   # 데이터 수신 전용 (GCS 동시 연결 시 권장)
MAVROS_MODE = 'full'       # 전체 기능 (제어 명령 포함)
```

## 발행 토픽 전체 목록

### /drone/* (mavros 기반 커스텀 토픽)
- 로컬좌표/목표: `local_ned/{x,y,z}`, `local_ned_target/{x,y,z}`
- 속도/목표: `local_ned_vel/{vx,vy,vz}`, `local_ned_vel_target/{vx,vy,vz}`
- 가속도: `accel/{x,y,z}`
- 자세/목표: `attitude/{roll,pitch,yaw}`, `attitude_target/{roll,pitch,yaw}`
- 각속도/목표: `attitude_rate/{roll,pitch,yaw}`, `attitude_rate_target/{roll,pitch,yaw}`
- 배터리: `battery/{voltage,current}`
- GPS: `gps/{latitude,longitude,altitude,ground_speed,course_angle}`
- Gyro: `gyro/{x,y,z}`
- 지자기: `mag/{x,y,z}`
- 모터 PWM: `rcout/{c1..c8,idle_pwm,max_pwm}`

### /respeaker/*
- `doa` (Int32, 0~359도), `vad` (Bool), `audio` (UInt8MultiArray, 16kHz PCM), `energy` (Float32, RMS)

### /thl100/*
- `temperature` (°C), `humidity` (%RH), `light` (Lux), `raw`

### /wcm6800/*
- `current` (A, DC는 부호 포함), `current_type` (AC/DC+/DC-), `raw`

## 데이터 녹화

```bash
ros2 bag record -a -o anomaly_data_$(date +%Y%m%d_%H%M%S)
```

## 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| ReSpeaker Pipe error | USB 인터페이스 충돌 | `respeaker_full_node.py`의 Tuning 클래스가 SEEED Control 인터페이스만 claim하는지 확인 |
| mavros 중복 실행 | 이전 프로세스 잔존 | `pkill -f mavros_node` 후 재실행 |
| rqt에서 mavros 토픽 안 보임 | QoS 불일치 (BEST_EFFORT) | `ros2 topic echo <topic> --qos-reliability best_effort` 사용 |
| USB 포트 번호 변경 | 재연결 시 ttyUSB/ttyACM 번호 변동 | `/dev/serial/by-id/` 고유 ID 사용 (이미 적용됨) |
| `ros2 launch` 실행 시 `KeyError: 'launch'` | `drone_sensors/launch/` 폴더에 `__init__.py`가 있어서 ROS2의 `launch` 모듈과 이름 충돌 | `rm src/drone_sensors/launch/__init__.py` 후 `build/install/log` 삭제하고 재빌드 |
