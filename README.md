# anomaly_sensor_ros2

드론 이상탐지(Anomaly Detection)를 위한 다중 센서 데이터 수집 ROS2 워크스페이스.

ReSpeaker 마이크, 온습도/조도 센서, 전류계, 비행 컨트롤러(FC) 데이터를 ROS2 토픽으로 통합 발행하고, rosbag 녹화 및 CSV/그래프 분석까지 한 번에 처리합니다.

## 구성

| 패키지 | 설명 | 발행 토픽 |
|---|---|---|
| `respeaker` | ReSpeaker Mic Array v3.0 (DoA/VAD/Audio/Energy) | `/respeaker/*` |
| `thl100_sensor` | OSTSen-THL100 온습도/조도 센서 (UART) | `/thl100/*` |
| `wcm6800_sensor` | Winson WCM6800 전류계 (UART) | `/wcm6800/*` |
| `drone_state` | mavros 데이터를 재가공한 커스텀 토픽 (FC 상태) | `/drone/*` |
| `drone_sensors` | mavros + 전체 센서 통합 launch 패키지 | - |

```
anomaly_sensor_ros2/
├── src/
│   ├── respeaker/
│   ├── thl100_sensor/
│   ├── wcm6800_sensor/
│   ├── drone_state/
│   └── drone_sensors/
│       └── launch/drone_sensor_launch.py
├── scripts/
│   ├── record_data.sh      # rosbag 녹화
│   └── analyze_bag.py      # CSV 변환 + 그래프 생성
├── install.sh              # 전체 환경 자동 설치
└── README.md
```

## 하드웨어 요구사항

- ReSpeaker Mic Array v3.0 (USB)
- OSTSen-THL100 온습도/조도계 (UART → USB, 9600bps)
- Winson WCM6800 전류계 (UART → USB, 9600bps)
- ArduPilot FC (CubeOrange 등, USB 또는 TELEM)

## 동작 확인 환경

- Ubuntu 22.04 + ROS2 Humble (x86_64)
- Raspberry Pi 4 + Ubuntu Server 22.04 + ROS2 Humble (arm64)

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
- 예전 워크스페이스 잔재가 `.bashrc`에 남아있다면 자동 정리
- ROS2 워크스페이스 빌드 (이전 빌드 산물 정리 후 진행)
- 편의 alias 등록 (`start_drone`, `record_drone` 등)

설치 후 **로그아웃 → 재로그인** 필요 (dialout 그룹 권한 적용).

## USB 장치 ID 확인

장치마다 USB 포트 번호(`ttyUSB0`, `ttyACM0` 등)는 재연결 시 바뀔 수 있으므로 `/dev/serial/by-id/` 고유 ID를 사용합니다.

```bash
check_usb   # 또는: ls -la /dev/serial/by-id/
```

장치 ID는 PC를 바꿔도 동일하지만(장치 자체의 시리얼번호 기반), 처음 설정하는 PC라면 확인된 ID를 `src/drone_sensors/launch/drone_sensor_launch.py`에 반영합니다.

```python
# FC 연결
FCU_URL = '/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_xxxxxxxx-if00:115200'

# THL100, WCM6800 포트 (DeclareLaunchArgument 기본값)
thl100_port_arg ...  default_value='/dev/serial/by-id/usb-Prolific_xxxx...'
wcm6800_port_arg ... default_value='/dev/serial/by-id/usb-Diwell_xxxx...'
```

수정 후 재빌드:
```bash
cd ~/anomaly_sensor_ros2
colcon build --packages-select drone_sensors --symlink-install
```

## FC 데이터 스트림 설정 (최초 1회)

ArduPilot은 기본적으로 일부 메시지만 전송합니다. mavros로 IMU/포지션 등 전체 데이터를 받으려면 SR 파라미터를 설정해야 합니다(FC에 영구 저장되므로 최초 1회만 필요).

```bash
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 "{param_id: 'SR0_RAW_SENS', value: {integer_value: 10}}"
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 "{param_id: 'SR0_EXT_STAT', value: {integer_value: 10}}"
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 "{param_id: 'SR0_RC_CHAN', value: {integer_value: 10}}"
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 "{param_id: 'SR0_POSITION', value: {integer_value: 10}}"
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 "{param_id: 'SR0_EXTRA1', value: {integer_value: 10}}"
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 "{param_id: 'SR0_EXTRA2', value: {integer_value: 10}}"
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 "{param_id: 'SR0_EXTRA3', value: {integer_value: 10}}"
```

`start_drone` 실행 중 `/mavros/imu/data` 등에 데이터가 안 보이면 이 단계를 먼저 확인하세요.

## 실행

```bash
start_drone     # mavros + 전체 센서 실행
```

새 터미널에서 토픽 확인:
```bash
check_topics
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

`src/drone_sensors/launch/drone_sensor_launch.py` 상단에서 설정합니다.

```python
MAVROS_MODE = 'readonly'   # 데이터 수신 전용 (GCS 동시 연결 시 권장, FC에 명령 안 보냄)
MAVROS_MODE = 'full'       # 전체 기능 (제어 명령 포함, 단독 운용 시)
```

## 데이터 녹화

```bash
record_drone 30     # 30초 녹화
record_drone         # 무제한 (Ctrl+C로 종료)
```

`~/anomaly_data/anomaly_data_YYYYMMDD_HHMMSS/` 에 저장됩니다.

기본 녹화 토픽은 `scripts/record_data.sh`의 `TOPICS` 배열에서 추가/제거할 수 있습니다. `/respeaker/audio`는 용량이 커서 기본 제외되어 있습니다.

## 데이터 분석

```bash
analyze_drone ~/anomaly_data/anomaly_data_20260616_112216
```

결과물:
```
anomaly_data_20260616_112216_csv/          # 토픽별 개별 CSV
anomaly_data_20260616_112216_merged.csv    # 타임스탬프 기준 병합 통합 테이블
anomaly_data_20260616_112216_overview.png  # 핵심 지표 그래프
```

터미널에는 토픽별 발행 Hz, min/max/mean 통계도 함께 출력됩니다.

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

## 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| ReSpeaker USB Pipe error | USB 인터페이스 충돌 (ALSA 오디오 인터페이스와 control transfer 동시 점유) | `respeaker_full_node.py`의 Tuning 클래스가 SEEED Control 인터페이스만 별도로 claim하고 있어야 함(이미 적용됨). `set_configuration()`/`detach_kernel_driver()` 호출하지 않을 것 |
| `ros2 launch` 실행 시 `KeyError: 'launch'` | `drone_sensors/launch/` 폴더에 `__init__.py`가 있어서 ROS2의 `launch` 모듈과 이름 충돌 | `rm src/drone_sensors/launch/__init__.py` 후 `build/install/log` 삭제하고 재빌드 (install.sh가 자동 처리) |
| mavros 중복 실행 시 크래시 (`invalid allocator`) | 이전 mavros 프로세스가 종료되지 않은 채 새로 실행됨 | `pkill -f mavros_node` 후 재실행. `start_drone` alias가 자동으로 처리함 |
| mavros를 launch 파일에 직접 Node로 추가하면 크래시 | launch가 `-r __node:=mavros`로 강제 노드 이름 재지정을 시도해 충돌 | `IncludeLaunchDescription` + `AnyLaunchDescriptionSource`로 mavros의 `apm.launch`(XML)를 include하는 방식 사용(이미 적용됨) |
| rqt 또는 직접 구독 시 mavros 토픽 안 보임 | QoS 불일치 (mavros는 BEST_EFFORT) | `ros2 topic echo <topic> --qos-reliability best_effort` 사용, 또는 구독 노드에서 동일 QoS 프로파일 적용 |
| `/mavros/imu/data` 등 일부 토픽이 발행 안 됨 | ArduPilot의 SR 스트림 파라미터 미설정 | 위 "FC 데이터 스트림 설정" 단계 진행 |
| USB 포트 번호 변경 | 재연결 시 ttyUSB/ttyACM 번호가 매번 달라짐 | `/dev/serial/by-id/` 고유 ID 사용 (이미 적용됨) |
| `pip install --break-system-packages` 옵션 오류 | pip 버전이 낮아 해당 옵션 미지원 (Ubuntu 22.04 기본 pip 22.0.2) | install.sh가 pip을 먼저 업그레이드하고 `~/.local/bin`을 PATH에 등록함(이미 적용됨) |
| `colcon: command not found` | `python3-colcon-common-extensions` 미설치 | install.sh의 시스템 의존성 목록에 포함됨(이미 적용됨) |
| `colcon build` 시 `File exists` 에러 (특정 패키지) | 이전 빌드의 `build/install` 잔여 파일과 충돌 | install.sh가 빌드 전 `build/install/log`를 정리함(이미 적용됨). 수동 빌드 시 `rm -rf build/<pkg> install/<pkg>` 후 재시도 |
| GitHub 저장소 클론 후 특정 패키지 폴더가 비어 있음 | 해당 패키지가 git submodule로 등록되어 일반 클론으로 내용이 안 받아짐 | `git rm --cached <pkg>` → `rm -rf <pkg>/.git` → `git add <pkg>` 로 일반 디렉토리로 전환 |
| matplotlib import 시 NumPy 관련 `ImportError`/`_ARRAY_API not found` | NumPy 2.x와 시스템 matplotlib(NumPy 1.x 빌드) 비호환 | `pip install "numpy<2"` (install.sh에 이미 반영) |
| 그래프 생성 시 `Multi-dimensional indexing... no longer supported` | 최신 pandas Series를 matplotlib에 직접 전달 | `analyze_bag.py`에서 `.to_numpy()`로 변환 후 plot (이미 적용됨) |
| THL100 파싱 시 "필드 수 오류" (패킷이 붙어서 들어옴) | UART 버퍼에 두 패킷이 겹쳐 수신됨 | `@` 기준으로 마지막 완전한 패킷만 추출하도록 파싱 로직 보완 필요 |
| FC 연결 끊김 (`mavconn: serial0: write: No such device`) | USB 케이블 분리 또는 FC 재부팅 | USB 재연결 후 `stop_drone` → `start_drone` |
| `record_drone`/`analyze_drone` alias가 안 먹음 | 이전 `.bashrc` 설정이 새 환경에 반영 안 됨 | 새 터미널 열거나 `source ~/.bashrc` 실행 |
| 새 PC/SD카드로 이전 후 `ros2_ws/install/setup.bash: No such file or directory` | 예전 워크스페이스(`~/ros2_ws`) source 줄이 `.bashrc`에 남아있음 | install.sh가 해당 줄을 자동 정리함(이미 적용됨) |
