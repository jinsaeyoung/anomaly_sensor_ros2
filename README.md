# anomaly_sensor_ros2

드론 이상탐지(Anomaly Detection)를 위한 다중 센서 데이터 수집 ROS2 워크스페이스.

MAVROS 원본 토픽을 변환 없이 그대로 녹화하고, UART 센서(온습도/전류)는 수신과 발행을 분리해 지정 주기로 발행합니다. FC의 arm/disarm에 맞춰 자동 녹화되며, 전원을 켜면 systemd가 전체 시스템을 자동으로 기동합니다.

---

## 목차

- [빠른 시작](#빠른-시작) — 처음 설치하는 경우 여기부터
- [시스템 구조](#시스템-구조)
- [패키지 구성](#패키지-구성)
- [편의 alias](#편의-alias)
- [장치 경로 설정](#장치-경로-설정)
- [FC 연결 설정](#fc-연결-설정) — SR 파라미터, 배선, 플러그인
- [실행](#실행)
- [온보드 무인 운용](#온보드-무인-운용-자동-녹화--부팅-자동-실행) — 자동 녹화, systemd
- [UART 센서 발행 구조](#uart-센서-발행-구조)
- [녹화 토픽 (42개)](#녹화-토픽-42개)
- [데이터 녹화 (수동)](#데이터-녹화-수동)
- [시간 동기화](#시간-동기화)
- [데이터 분석](#데이터-분석) — CSV, 추종 오차, 오디오 추출
- [테스트](#테스트)
- [MAVLink 경로 한계](#mavlink-경로-한계)
- [정상 동작 확인](#정상-동작-확인)
- [트러블슈팅](#트러블슈팅)

---

## 빠른 시작

처음 설치하는 경우 아래 순서대로 진행하세요. 전체 약 40~60분이 소요됩니다.

### 필요 환경

| 항목 | 요구사항 |
|---|---|
| OS | Ubuntu 22.04 LTS (x86_64 또는 arm64) |
| ROS | ROS2 Humble |
| 검증 환경 | x86_64 PC, Raspberry Pi 4, Mobilint NPU 모듈 |

### 하드웨어

| 장치 | 연결 | 비고 |
|---|---|---|
| ArduPilot FC (CubeOrange 등) | TELEM2 + USB-TTL 젠더(CH340) @921600 | USB 직결도 가능 |
| ReSpeaker Mic Array v3.0 | USB | |
| OSTSen-THL100 온습도/조도계 | UART → USB, 9600bps | |
| Winson WCM6800 전류계 | UART → USB, 9600bps | |

USB 장치가 많으므로 **외부 전원 공급형 USB 허브**를 권장합니다.

---

### 1단계 — ROS2 Humble 설치

이미 설치되어 있으면 건너뜁니다. `install.sh`가 ROS2 apt 저장소를 확인하고, 없거나 키가 만료됐으면 자동으로 설정합니다.

```bash
ls /opt/ros/humble/setup.bash        # 있으면 2단계로
```

새로 설치할 때는 공식 문서의 현재 방식(ros2-apt-source 패키지)을 따릅니다. 예전의 키 파일을 직접 받는 방식은 키가 만료되면 패키지를 찾지 못합니다.

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && sudo apt install -y ros-humble-ros-base ros-dev-tools
```

### 2단계 — 저장소 클론 및 설치

**처음 설치하는 경우**

```bash
cd ~
git clone https://github.com/jinsaeyoung/anomaly_sensor_ros2.git
cd anomaly_sensor_ros2
bash install.sh
```

**이미 폴더가 있는 경우**

`대상 경로 'anomaly_sensor_ros2'이(가) 이미 있고 빈 디렉터리가 아닙니다` 오류가 나면 아래 중 하나를 선택하세요.

기존 내용을 버리고 새로 받으려면:

```bash
cd ~
sudo systemctl stop anomaly-sensor 2>/dev/null || true
rm -rf anomaly_sensor_ros2
git clone https://github.com/jinsaeyoung/anomaly_sensor_ros2.git
cd anomaly_sensor_ros2
bash install.sh
```

> 녹화 데이터(`~/anomaly_data`)는 별도 경로이므로 삭제되지 않습니다.
> 다만 launch 파일의 장치 ID 등을 직접 수정했다면 그 내용도 사라지니 미리 백업하세요.

기존 폴더를 유지하며 최신으로 갱신하려면:

```bash
update_drone          # (설치 후 사용 가능) pull → 패키징 → 빌드 → 테스트 → 재시작
```

처음 설치하는 모듈이라 `update_drone`이 아직 없다면 `git pull` 후 `bash install.sh`를 실행하세요.

`install.sh`가 의존성 설치(mavros, pyserial, pandas 등), 장치 권한, 워크스페이스 빌드, 편의 alias 등록까지 처리합니다. 여러 번 실행해도 안전합니다.

> **`sudo bash install.sh`로 실행하지 마세요.** 파일이 root 소유가 되고 alias가 root 계정에 등록되어 이후 계속 권한 문제가 생깁니다. 일반 사용자로 실행하면 필요한 단계에서만 비밀번호를 묻습니다. 예전에 sudo로 실행한 흔적이 있으면 자동으로 정리합니다.

#### 장치 권한

USB 시리얼 장치(FC·THL100·WCM6800 젠더)와 ReSpeaker에 udev 규칙으로 직접 권한을 주므로, **설치 직후 재로그인 없이** 바로 쓸 수 있습니다. 그룹(`dialout`, `audio`)만으로는 추가 후 재로그인해야 적용되어, 새 모듈에서 설치 직후 확인 명령이 `Permission denied`로 실패했습니다.

```bash
fix_permissions          # 권한 설정 + 점검 (문제가 생겼을 때 다시 실행)
fix_permissions check    # 점검만
```

이 모듈은 데이터 수집 전용 장비이므로 장치 권한을 여는 쪽을 택했습니다.

### 3단계 — 온보드 환경 설정

```bash
bash scripts/setup_onboard_env.sh
```

무인 운용에 필요한 시스템 설정을 적용합니다.

| 항목 | 이유 |
|---|---|
| `brltty` 제거 | CH340 젠더를 점자 장치로 오인해 가로채는 문제 방지 |
| 이전 sudo 예외 제거 | 예전 버전의 비밀번호 생략 설정 정리 (더 이상 불필요) |
| `ROS_DOMAIN_ID=0` | 서비스와 셸의 DDS 도메인 일치 |
| `dialout` 그룹 | 시리얼 포트 권한 |
| 로그 파일 소유권 | systemd가 root로 만드는 문제 방지 |

> **brltty를 제거했다면 FC 젠더 USB를 한 번 뽑았다 다시 꽂으세요.** 그래야 시리얼 노드가 생성됩니다.

### 4단계 — 재로그인

`dialout` 그룹 적용에 필요합니다.

```bash
exit
```

재접속 후 확인합니다.

```bash
onboard_env check        # 모든 항목 ✅ 확인
detect_serial            # FC / THL100 / WCM6800 판별
```

```bash
detect_serial            # 어떤 포트가 어느 장치인지 자동 판별
```

세 장치가 모두 확정되면 정상입니다. launch가 실행될 때도 같은 탐색을 자동으로 수행하므로, **젠더를 교체하거나 포트 순서가 바뀌어도 별도 수정이 필요 없습니다.**

### 5단계 — FC 파라미터 설정 (최초 1회)

Mission Planner 또는 QGroundControl의 Full Parameter List에서 설정합니다. FC에 영구 저장되므로 기체당 한 번만 하면 됩니다.

```
SERIAL2_PROTOCOL = 2      # MAVLink2
SERIAL2_BAUD     = 921    # 921600
SERIAL2_OPTIONS  = 0      # 흐름제어 OFF

SR2_RAW_SENS = 50    SR2_EXTRA1 = 50
SR2_RAW_CTRL = 50    SR2_RC_CHAN = 50
SR2_POSITION = 30    SR2_EXTRA3 = 20
SR2_EXT_STAT = 10    SR2_EXTRA2 = 10
```

USB 직결이면 `SERIAL0_*`, `SR0_*`를 사용합니다.

`SYSID_THISMAV`는 **바꿀 필요가 없습니다.** 2~254여도 HEARTBEAT로 자동 탐지합니다.

### 6단계 — 수동 실행 검증

서비스로 등록하기 전에 직접 실행해 확인합니다.

```bash
ros2 launch drone_sensors drone_sensor_launch.py use_auto_record:=true
```

새 터미널에서 상태를 봅니다.

```bash
monitor_drone --once
```

```
┌─ FC 상태 ─────────────────────────────────────────────
│  연결: 연결됨  상태: disarmed          모드: STABILIZE
└───────────────────────────────────────────────────────
┌─ 녹화 상태 ───────────────────────────────────────────
│  ○  대기 중 (arm 하면 자동 시작)
└───────────────────────────────────────────────────────
```

주요 토픽 주기를 확인합니다.

```bash
timeout 8 ros2 topic hz /mavros/imu/data                   # 50Hz
timeout 8 ros2 topic hz /mavros/vibration/raw/vibration     # 20Hz
timeout 8 ros2 topic hz /respeaker/audio                    # 15.6Hz
timeout 8 ros2 topic hz /thl100/data                        # 1Hz
```

### 7단계 — 녹화·분석 검증

```bash
record_drone 30
ros2 bag info ~/anomaly_data/anomaly_data_*
analyze_drone ~/anomaly_data/anomaly_data_*
```

`~/anomaly_data/analyzed/<bag이름>/`에 CSV와 그래프가 생성됩니다.

### 8단계 — 서비스 등록

수동 실행을 중지(Ctrl+C)한 뒤 등록합니다.

```bash
bash scripts/install_service.sh          # 등록만 (부팅 자동실행 OFF)
sudo systemctl start anomaly-sensor
sleep 40
monitor_drone --once
```

### 9단계 — 부팅 자동 실행

실기체 운용 준비가 끝나면 활성화합니다.

```bash
bash scripts/install_service.sh enable
sudo reboot
```

재부팅 후 아무 조작 없이 확인합니다.

```bash
sleep 60
monitor_drone --once
head -40 ~/anomaly_data/onboard.log
```

FC 연결과 녹화 대기 상태가 자동으로 잡히면 완료입니다. 이제 arm하면 녹화가 시작되고 disarm하면 종료됩니다.

---

### 문제가 생기면

| 증상 | 확인 |
|---|---|
| `detect_serial`에 `권한 없음` / `Permission denied` | 장치 권한 미설정 (새 모듈, 설치 중단 등) | `fix_permissions` — 재로그인 불필요. 서비스는 영향 없음 |
| `detect_serial`이 모든 포트를 '사용 중'으로만 표시 | 서비스 실행 중 (이전 버전) | 점유 프로세스로 판별해 표시 (적용됨) |
| `Unable to locate package ros-humble-mavros` (다른 ROS 패키지는 설치됨) | **2026-09 현재 Humble 저장소에서 mavros·mavros_extras·libmavconn이 빠짐** ([mavlink/mavros#2293](https://github.com/mavlink/mavros/issues/2293)) | `install.sh`가 ROS 공식 스냅샷(2026-08-07)에서 네 패키지를 같은 버전으로 설치하고 고정(hold). 다른 날짜: `MAVROS_SNAPSHOT=YYYY-MM-DD bash install.sh` |
| `Unable to locate package ros-humble-*` (전부) | ROS2 apt 저장소가 없거나, 예전 방식의 키가 만료됐거나, **같은 저장소가 다른 키로 중복 등록**됨 | `install.sh`가 자동 처리: 공식 방식(ros2-apt-source)으로 설정 → 중복 항목은 `.disabled-by-anomaly`로 이름을 바꿔 보관(삭제 아님) → 목록 재수신. 그래도 안 되면 apt의 실제 오류와 등록된 ROS 저장소를 보여줍니다 |
| 저장소가 정상인데 `ROS 패키지를 찾을 수 없습니다` (한국어 PC) | apt 출력이 언어를 따라 `Candidate:` → `후보:`로 바뀌어 판정 실패 (이전 버전) | 출력 해석 시 언어를 영어로 고정 (적용됨) |
| `monitor_drone`이 창 없이 바로 터미널로 돌아옴 (이전 버전) | 첫 화면에서 녹화 상태가 아직 없을 때 '전원 차단' 표시가 오류를 냈고, 별도 화면 때문에 메시지도 사라짐 | 수정됨. 이제 한 칸에서 오류가 나도 그 칸에만 표시되고, 종료될 때는 원래 화면으로 돌아온 뒤 오류를 보여줌 |
| `update_drone: 명령을 찾을 수 없습니다` | `update_drone` 추가 전에 설치한 모듈 | `bash install.sh && source ~/.bashrc` 한 번 실행 |
| 설치 직후 시리얼 권한 오류 | `fix_permissions` 실행 (재로그인 불필요) |
| `monitor_drone`에 아무것도 안 보임 | `echo $ROS_DOMAIN_ID` 가 `0` 인지 |
| FC만 연결 안 됨 | `detect_serial`로 인식 확인, brltty 제거 여부 |
| 특정 토픽만 안 옴 | SR2 파라미터 설정 확인 |

자세한 내용은 아래 [트러블슈팅](#트러블슈팅) 절을 참고하세요.

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
| `drone_sensors` | 통합 launch + `fcu_manager_node`(mavros 관리) · `sensor_health_node` · `auto_record_node` |

```
anomaly_sensor_ros2/
├── src/
│   ├── respeaker/respeaker/respeaker_full_node.py      # 마이크 (장치 대기·재연결)
│   ├── thl100_sensor/thl100_sensor/thl100_uart_node.py # 온습도/조도 (포트 재탐색)
│   ├── wcm6800_sensor/wcm6800_sensor/wcm6800_uart_node.py # 전류계 (포트 재탐색)
│   └── drone_sensors/
│       ├── launch/drone_sensor_launch.py
│       ├── config/apm_pluginlists.yaml   # mavros 플러그인 (vibration 활성화)
│       └── drone_sensors/
│           ├── fcu_manager_node.py       # mavros 기동·감시·재연결 (포트/baud/SYSID)
│           ├── sensor_health_node.py     # 센서 상태 5초 주기 발행
│           └── auto_record_node.py       # arm/disarm 자동 녹화
├── scripts/
│   ├── serial_autodetect.py   # 장치 판별 (데이터 형식 + MAVLink CRC/HEARTBEAT)
│   ├── monitor_node.py        # 실시간 모니터 (monitor_drone)
│   ├── record_data.sh         # rosbag 수동 녹화
│   ├── analyze_bag.py         # CSV 변환 + 10Hz 정렬 + 그래프
│   ├── extract_audio.py       # 마이크 원본 PCM → WAV
│   ├── verify_bag.py          # 녹화 검증 (드론 판정 + 외부센서 표기)
│   ├── extract_bag_log.sh     # 특정 bag 전후 로그 추출
│   ├── start_onboard.sh       # 온보드 자동 실행 (systemd 호출)
│   ├── install_service.sh     # 부팅 자동 실행 서비스 등록
│   ├── setup_onboard_env.sh   # 온보드 환경 일괄 설정
│   ├── setup_permissions.sh   # 장치 권한 (udev·그룹·소유권) — fix_permissions
│   ├── check_time_sync.sh     # 시간 동기화 확인
│   ├── guard_service.sh       # 수동 실행 시 서비스 충돌 방지
│   └── watch_fcu.sh           # fc_status 본체
├── tests/
│   ├── test_parsers.py        # 파서·SYSID 탐지·좌표변환 단위 테스트
│   └── virtual_uart_test.sh   # 가상 UART 통합 테스트
├── fix_packaging.sh           # ROS2 패키지 구조 표준화
├── install.sh                 # 전체 환경 자동 설치
├── .gitignore
└── README.md
```

---

## mavros 설치 관련 (2026-09 현재)

Humble(Jammy) apt 저장소에서 `ros-humble-mavros`, `ros-humble-mavros-extras`, `ros-humble-libmavconn`이 빠지고 `ros-humble-mavros-msgs`(2.15.1)만 남아 있습니다([mavlink/mavros#2293](https://github.com/mavlink/mavros/issues/2293)). 새 모듈에서는 `apt install ros-humble-mavros`가 실패합니다.

`install.sh`가 이를 자동으로 처리합니다.

| 상황 | 동작 |
|---|---|
| mavros 이미 설치됨 | 그대로 사용. 저장소에 mavros가 없으면 **현재 버전 고정(hold)** — `apt upgrade`가 mavros_msgs만 올려 짝이 깨지는 것을 방지 |
| 저장소에 mavros 있음 | 정상 설치 |
| 저장소에 mavros 없음 | ROS 공식 스냅샷(`snapshots.ros.org`, 2026-08-07)을 잠시 추가해 **네 패키지를 같은 버전으로** 설치하고 고정. 스냅샷 목록은 바로 제거 |

mavros는 mavros_msgs와 함께 빌드된 짝이라, mavros만 옛 버전이고 mavros_msgs가 최신이면 실행 중 문제가 생길 수 있습니다. 그래서 네 패키지를 함께 맞추고 고정합니다.

저장소가 복구되면 고정을 풀고 올리면 됩니다.

```bash
sudo apt-mark unhold ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs ros-humble-libmavconn
sudo apt update && sudo apt install --only-upgrade ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs ros-humble-libmavconn
```

---

## 편의 alias

`install.sh`가 `.bashrc`에 등록합니다. 새 터미널을 열거나 `source ~/.bashrc` 후 사용하세요.

| 구분 | alias | 설명 |
|---|---|---|
| 실행 | `start_drone` | 수동 실행 (시간 동기화 확인 → launch, 자동녹화 없음) |
| | `stop_drone` | 전체 종료 |
| 상태 | `monitor_drone` | 실시간 모니터 — FC·녹화·센서·로그·최근 녹화 (`--interval 1`, `--no-log`, `--once`) |
| | `fc_status` | FC 관리 상태 1회 (포트·baud·SYSID·재시작 사유) |
| | `onboard_log` | 서비스 실행 로그 실시간 |
| | `service_status` | 부팅 자동실행 여부 |
| 장치 | `detect_serial` | FC·THL100·WCM6800 판별 (FC baud·SYSID·링크 구성원 포함) |
| | `onboard_env` | 온보드 환경 설정 / `onboard_env check` 로 점검 |
| | `fix_permissions` | 장치 권한 설정·점검 — udev·그룹·소유권 (재로그인 불필요) |
| 녹화·분석 | `record_drone` | 수동 녹화 (`record_drone 30`) |
| | `verify_bag` | 녹화 검증 — 드론 판정 + 외부센서 표기 (`--all`, `--csv`) |
| | `bag_log` | 특정 bag 녹화 전후 로그 |
| | `analyze_drone` | CSV + 그래프 |
| | `extract_audio` | 마이크 원본 PCM → WAV |
| 배포 | `update_drone` | GitHub 최신 반영 → 패키징 → 빌드 → 테스트 → 서비스 재시작 |

이전 버전의 `check_topics`, `check_usb`, `check_record`, `watch_fcu`, `detect_fc`, `scan_bags`, `monitor_fast`, `monitor_only`는 위 명령으로 통합되었습니다. `install.sh`를 다시 실행하면 자동으로 정리됩니다.

---

## 코드 갱신 — `update_drone`

여러 모듈에 같은 코드를 배포할 때 파일을 하나씩 복사하면 빠뜨리기 쉽습니다. `update_drone`은 저장소 전체를 한 번에 맞춥니다.

```bash
update_drone              # GitHub 최신 반영
update_drone --no-pull    # 로컬 수정 후 다시 빌드만
```

| 단계 | 내용 |
|---|---|
| 1. 받기 | `git pull --ff-only`. **로컬 수정이 있으면 받지 않고 멈춤** (덮어쓰기 방지) |
| 2. 패키징·빌드 | `fix_packaging.sh` → `colcon build` |
| 3. 테스트 | `tests/test_parsers.py`. **실패하면 서비스를 재시작하지 않음** — 기존 코드로 계속 수집 |
| 4. 재시작 | 서비스가 실행 중이었을 때만 |

`install.sh`가 바뀐 경우(새 명령 추가 등) 안내가 나오면 `bash install.sh && source ~/.bashrc`를 한 번 실행하세요.

### 패키지 안의 사본

`src/drone_sensors/scripts/`의 `serial_autodetect.py`, `verify_bag.py`는 노드가 불러오는 **설치용 사본**이며 원본은 `scripts/`입니다. 원본만 고치고 `fix_packaging.sh`를 돌리지 않은 채 커밋하면 실행 시 옛 코드가 쓰이므로, 테스트가 두 파일이 같은지 확인합니다. 원본을 고쳤다면 커밋 전에 `bash fix_packaging.sh`를 실행하세요.

---

## 장치 자동 구분과 FC 자동 연결

**어떤 USB-UART 젠더를 어느 USB 포트에 꽂아도 됩니다.** 설치한 하드웨어마다 경로를 따로 맞출 필요가 없습니다.

### 장치 구분 방식

VID:PID나 by-id 경로에 의존하지 않고, 각 포트를 열어 **수신 데이터의 형식**으로 판별합니다. 같은 모델 젠더를 여러 개 써도, 젠더를 바꿔 끼워도 구분됩니다.

| 장치 | 판별 기준 |
|---|---|
| FC | MAVLink 프레임 **CRC 검증** + HEARTBEAT 해석 (baud·SYSID 자동) |
| THL100 | 9600bps, `@ID,seq,temp,humi,light` |
| WCM6800 | 9600bps, `[~+-]NNNNN` 6바이트 |
| ReSpeaker | USB ID `2886:0018` (오디오 장치) |

다른 프로세스가 이미 열고 있는 포트는 탐색에서 제외합니다. 실행 중인 노드의 포트를 열면 데이터를 빼앗아 양쪽이 모두 깨지기 때문입니다. 포트는 `exclusive` 모드로 열어 동시 점유도 막습니다.

관리 노드는 mavros를 띄우기 **직전에** FC 포트를 선점 표시(`/tmp/anomaly_sensor_claims-<uid>/`, 사용자별)합니다. mavros가 포트를 열기까지의 짧은 틈에 다른 노드가 먼저 열면 HEARTBEAT를 놓쳐 첫 연결이 실패하기 때문입니다. 표시한 프로세스가 죽으면 표시는 자동으로 무시됩니다.

센서 탐지에 실패하면 임의의 경로를 넣지 않고 `auto`로 두어 노드가 바로 재탐색합니다. 추측한 경로가 FC 포트와 겹치면 mavros 데이터를 빼앗을 수 있기 때문입니다.

```bash
detect_serial     # 전체 장치 판별
```

서비스가 실행 중이면 모든 포트를 노드가 쓰고 있습니다. 이때는 포트를 열지 않고 **포트를 쓰는 프로세스로 판별**합니다(`thl100_node`가 잡은 포트 = THL100, `mavros_node`가 잡은 포트 = FC). 실행 중인 포트를 열면 데이터를 빼앗아 녹화가 깨지기 때문입니다. FC의 baud·SYSID는 이 경우 `fc_status`로 확인하세요.

```
  ✅ OSTSen-THL100 (온습도/조도)
      /dev/ttyUSB1  (thl100_node 가 사용 중 — 정상 동작)
```

```
  포트      : /dev/ttyUSB2
  baud      : 921600
  대상 기체 : SYSID 7  COMPID 1
  프로토콜  : MAVLink v2.0
  mavros ID : 7.191  (컴패니언 규약)
  링크의 다른 구성원 (대상에서 제외됨):
      255.190  GCS
      7.154    GIMBAL
```

### 실행 중 재연결

| 장치 | 방식 |
|---|---|
| FC | `fcu_manager_node`가 mavros를 자식 프로세스로 관리. 연결이 끊기면 mavros만 내리고 **포트·baud·SYSID를 다시 탐지**해 새 설정으로 재기동 |
| THL100 / WCM6800 | 정상 패킷이 일정 시간(15초/8초) 끊기면 포트를 놓고 **비어 있는 포트에서 자기 장치를 다시 탐색** |
| ReSpeaker | 장치가 없어도 노드가 대기하며 3초마다 재연결 |

mavros는 시작할 때 경로와 SYSID를 고정하고 바꾸지 않습니다. USB 재삽입으로 `ttyUSB0`이 `ttyUSB2`로 바뀌거나 기체를 교체하면 스스로 복구하지 못하므로, 바깥에서 새 설정을 찾아 다시 띄워야 합니다. 이때 **센서 노드와 녹화 프로세스는 건드리지 않습니다.** rosbag은 토픽 이름으로 구독하므로 mavros가 다시 뜨면 같은 bag에 FC 데이터가 이어서 기록됩니다.

UART 노드가 재삽입 후 다른 장치의 포트(예: FC)를 잡게 되는 경우도 처리합니다. 자기 형식의 패킷이 오지 않으면 그 포트를 놓아주므로, FC 관리 노드가 해당 포트를 찾아갈 수 있습니다.

```bash
fc_status          # 관리 상태 (연결/복구/재시작 횟수)
```

### SYSID 자동 탐지

FC의 `SYSID_THISMAV`가 1이 아니어도(2~254) **FC를 수정하지 않고 연결됩니다.**

FC는 1초마다 HEARTBEAT를 보내고, 그 안에 자신의 SYSID가 들어 있습니다. 이것을 **수신만** 해서 읽으므로 SYSID를 1부터 255까지 하나씩 시도할 필요가 없고, 탐지 과정에서 **FC로 아무것도 송신하지 않습니다.** GCS·페이로드·FC 동작에 영향을 주지 않습니다.

기체를 지정하고 싶으면 인자로 고정할 수 있습니다.

```bash
ros2 launch drone_sensors drone_sensor_launch.py tgt_system:=7
TGT_SYSTEM=7 bash scripts/install_service.sh          # 서비스 기본값
```

### GCS·페이로드·다중 기체와의 관계

| 항목 | 처리 |
|---|---|
| **GCS** (Mission Planner 등) | HEARTBEAT `type=GCS`, `autopilot=INVALID` → 대상에서 제외 |
| **짐벌·카메라·기타 페이로드** | `autopilot=INVALID` → 제외. 같은 기체 SYSID를 쓰더라도 compid로 구분 |
| **다른 컴패니언 컴퓨터** | `type=ONBOARD_CONTROLLER` → 제외. 이미 compid 191을 쓰고 있으면 mavros는 194/195/196 중 빈 번호 사용 |
| **mavros 자기 ID** | `system_id = 대상 SYSID`, `component_id = 191` — MAVLink 컴패니언 규약. GCS에는 "기체 N의 부속 장치"로 보이고 별도 기체로 오인되지 않음 |
| **SYSID 255** | 사용하지 않음. GCS 관례 번호이며 ArduPilot `SYSID_MYGCS`(기본 255)와 겹치면 GCS 페일세이프·RC 오버라이드 판정에 영향 |
| **다중 기체 링크** | 비행제어기 HEARTBEAT가 여러 SYSID로 보이면 가장 자주 보인 기체를 고르고 **경고**. 이 경우 `tgt_system`을 지정 권장 |
| **MAVLink 서명(signing)** | 탐지는 되며 수신 데이터는 기록됨. 다만 mavros의 요청(파라미터 조회 등)은 FC가 거부할 수 있어 경고 표시 |
| **MAVLink1 전용 FC** | 프레임 형식으로 판별해 `fcu_protocol=v1.0` 자동 설정 |

잡음이나 잘못된 baud에서 읽은 데이터로 엉뚱한 SYSID가 잡히지 않도록 **모든 프레임의 CRC를 검증**합니다. (이전 로그의 `detected remote address 191.239` 같은 값이 이런 잡음에서 나온 것입니다)

### baud 자동 탐지

ArduPilot `SERIALn_BAUD`로 흔히 쓰는 값을 차례로 시도합니다.

```
921600 → 115200 → 57600 → 460800 → 230400 → 500000
```

CRC가 맞는 프레임이 나오는 속도를 채택하며, 신호 자체가 없는 포트는 첫 시도에서 바로 건너뜁니다.

### 비행 중 복구

기본값은 **비행 중에도 복구**합니다(`recover_while_armed=True`). mavros만 다시 띄우므로 센서 녹화는 끊기지 않고, FC 데이터가 이미 끊긴 상태라 재기동으로 잃을 것이 없기 때문입니다. mavros는 FC로 제어 명령을 보내지 않으므로 비행에도 영향이 없습니다.

비행 중 재기동을 원하지 않으면 `fcu_manager_node`의 `recover_while_armed`를 `false`로 두세요. 이 경우 착륙(disarm) 후 복구합니다.

### 수동 지정

자동 탐지보다 우선합니다.

```bash
ros2 launch drone_sensors drone_sensor_launch.py \
  fcu_url:=/dev/ttyACM0:115200 \
  tgt_system:=3 \
  thl100_port:=/dev/ttyUSB0 \
  wcm6800_port:=/dev/ttyUSB1
```

`fcu_url`을 지정해도 연결이 끊기면 재탐지합니다. 경로를 완전히 고정하려면 `fcu_rediscover:=false`를 추가하세요. 이전 방식(launch가 mavros를 직접 실행, 자동 복구 없음)은 `fcu_managed:=false`로 쓸 수 있습니다.

자동 탐지를 끄려면 `ANOMALY_AUTODETECT=0`을 씁니다. 이 경우 `drone_sensor_launch.py` 상단의 `DEFAULT_*` 값이 쓰이며, 각 노드가 실행 중 재탐색하므로 값이 틀려도 복구됩니다.

---

## FC 연결 설정

### 연결 방식

TELEM 포트에 **어떤 USB-UART 젠더(CH340·PL2303·CP2102 등)든** 연결할 수 있고, USB 직결도 됩니다. 포트·baud·SYSID는 자동 탐지됩니다.

| 연결 | 일반적인 baud | SR 파라미터 |
|---|---|---|
| **TELEM2** (권장) | 921600 | `SR2_*` |
| TELEM1 | 57600 | `SR1_*` |
| USB 직결 | 115200 | `SR0_*` |

**포트가 바뀌면 SR 파라미터 접두어도 바뀝니다.** USB로 `SR0_*`를 설정해두고 TELEM2로 옮기면 데이터가 오지 않으니 `SR2_*`를 다시 설정해야 합니다.

```bash
detect_serial                   # 연결된 장치 판별

ros2 launch drone_sensors drone_sensor_launch.py \
  fcu_url:=/dev/ttyACM0:115200  # 다른 포트/속도로 실행
```

연결 방식과 무관하게 포트·baud·SYSID는 자동 탐지됩니다. 아래 표는 FC 측 설정 참고용입니다.

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

### mavros 플러그인 설정

mavros 기본 설정(`/opt/ros/humble/share/mavros/launch/apm_pluginlists.yaml`)은 일부 플러그인을 `plugin_denylist`로 막아둡니다. 그중 두 개가 이 프로젝트에 필요합니다.

| 플러그인 | 토픽 | 용도 |
|---|---|---|
| `vibration` | `/mavros/vibration/raw/vibration` | 진동 — 모터 이상탐지 핵심 지표 |
| `altitude` | `/mavros/altitude` | 고도 상세 (AMSL, 지형 등) |

SR 파라미터를 설정해도 플러그인이 로드되지 않으면 토픽 자체가 생성되지 않습니다. 로그에 이렇게 남습니다.

```
[mavros.mavros]: Plugin vibration ignored
```

`src/drone_sensors/config/apm_pluginlists.yaml`에 두 플러그인을 활성화한 목록을 두고 launch가 이를 사용합니다. 시스템 파일을 직접 고치면 mavros 업데이트 시 되돌아가므로 패키지 안에서 관리합니다.

**apm.launch 대신 node.launch를 직접 include합니다.** `apm.launch`는 `pluginlists_yaml`을 인자로 선언하지 않고 하위 `node.launch`에 하드코딩해 전달하므로, 외부에서 값을 줘도 무시되기 때문입니다.

```xml
<!-- apm.launch — pluginlists_yaml 인자 선언이 없음 -->
<include file="$(find-pkg-share mavros)/launch/node.launch">
    <arg name="pluginlists_yaml" value="$(find-pkg-share mavros)/launch/apm_pluginlists.yaml" />
                                        ↑ 하드코딩
</include>
```

`node.launch`는 `pluginlists_yaml`, `config_yaml`을 모두 인자로 받으므로 우리 파일을 지정할 수 있습니다. `config_yaml`은 mavros 기본값(`apm_config.yaml`)을 그대로 씁니다.

확인 방법입니다.

```bash
grep -i "vibration" ~/anomaly_data/onboard.log | tail -3   # "ignored" 가 없어야 정상
timeout 8 ros2 topic hz /mavros/vibration/raw/vibration
```

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

보통은 `detect_serial`이 자동으로 찾습니다. 그래도 안 될 때 어떤 속도에서 신호가 오는지 직접 확인하는 방법입니다.

```bash
python3 -c "
import serial, time
PORT='/dev/ttyUSB0'          # ls /dev/ttyUSB* 로 확인한 경로
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

## 실행

```bash
start_drone
```

`start_drone`은 다음 순서로 동작합니다.

1. `check_time_sync.sh` — 시간 동기화 확인/보정
2. 이전 mavros 프로세스 정리
3. `ros2 launch drone_sensors drone_sensor_launch.py`

발행 주기를 변경하려면 인자를 넘깁니다.

```bash
ros2 launch drone_sensors drone_sensor_launch.py \
  thl100_rate:=1.0 \
  wcm6800_rate:=10.0 \
  respeaker_update_rate:=25.0
```

새 터미널에서 상태 확인:

```bash
monitor_drone
```

종료:

```bash
stop_drone
```

> `start_drone`에는 `use_auto_record:=true`가 없어 자동 녹화 노드가 뜨지 않습니다.
> 자동 녹화를 함께 쓰려면 인자를 명시하거나 systemd 서비스를 사용하세요.

---

## 온보드 무인 운용 (자동 녹화 + 부팅 자동 실행)

비행 중 사람이 개입하지 않고 데이터를 수집하는 구성입니다. FC의 arm 상태에 맞춰 녹화가 자동으로 시작·종료되고, 전원을 켜면 시스템이 스스로 준비를 마칩니다.

### 자동 녹화 동작

```
disarmed → armed   : 녹화 시작
armed → disarmed   : post_disarm_sec(기본 10초) 후 종료
재arm (종료 대기 중) : 종료 예약 취소, 같은 bag에 계속 기록
```

착륙 직후 데이터를 놓치지 않도록 disarm 후에도 잠시 더 기록하며, 연속 이륙 시에는 파일이 쪼개지지 않습니다.

### 실시간 모니터링

터미널 하나로 상태와 로그를 함께 봅니다.

```bash
monitor_drone              # 3초 갱신 (기본)
monitor_drone --interval 1 # 1초 갱신 (arm 테스트)
monitor_drone --interval 5 # 5초 갱신 (장시간 방치)
monitor_drone --log 10     # 로그 10줄
monitor_drone --no-log     # 로그 섹션 제외
monitor_drone --once       # 1회 출력
```

```
┌─ FC 상태 ──────────────────────────────────────────────
│  연결: 연결됨   상태: >>> ARMED <<<   모드: STABILIZE
│  포트 ttyUSB2 @ 921600   대상 1.1   자기ID 1.191
│  관리: 연결 유지   재시작 0회
└───────────────────────────────────────────────────────────
┌─ 녹화 상태 ────────────────────────────────────────────
│  ●  녹화 중        경과: 42초
│     파일: flight_20260921_190312
│     디스크 여유: 392.9 GB
└───────────────────────────────────────────────────────────
┌─ 센서 연결 ────────────────────────────────────────────
│  ● FC         정상           49.9Hz
│  ● ReSpeaker  정상           31.2Hz
│  ● THL100     정상            1.0Hz
│  ● WCM6800    정상           10.0Hz
└───────────────────────────────────────────────────────────
┌─ 최근 로그 (onboard.log) ──────────────────────────────
│ 19:03:12 [auto_record_node]: 프리플라이트: 전 센서 정상
│ 19:03:12 [auto_record_node]: 녹화 시작 (armed) → flight_...
└───────────────────────────────────────────────────────────
┌─ 최근 녹화 3건 ─────────────────────────────────────────
│  09-21 19:03  flight_20260921_190312      412.3MB  프리플라이트 OK
│  09-21 18:40  flight_20260921_184002      388.1MB  프리플라이트 경고
└───────────────────────────────────────────────────────────
```

| 섹션 | 내용 |
|---|---|
| FC 상태 | 연결·arm·모드, 관리 노드가 잡은 포트·baud·SYSID, 복구 횟수 |
| 녹화 상태 | 녹화 여부·경과·파일·디스크 여유 |
| 센서 연결 | 네 센서의 상태와 실측 Hz (`sensor_health_node` 5초 주기) |
| 최근 로그 | 의미 있는 이벤트 6줄. **최근 10분 이내만** 표시해 옛 로그가 섞이지 않음 |
| 최근 녹화 3건 | 시각·크기·녹화 시작 시 프리플라이트 결과 |

top/htop처럼 **별도 화면**을 쓰므로 갱신 내용이 스크롤백에 쌓이지 않고, 종료하면 원래 터미널 내용이 돌아옵니다. 터미널 크기에 맞춰 줄을 자르며, 창이 전체를 담기에 작으면 아래쪽에 안내가 표시됩니다(전체 약 40줄). 관리 노드가 mavros를 재시작한 적이 있으면 마지막 사유를 함께 보여줍니다.

모니터는 저주기 상태 토픽 4개만 구독하는 상주 노드라 부하가 거의 없습니다. 센서 Hz는 `sensor_health_node`가 계산한 값을 받아 쓰므로 모니터가 50Hz 토픽을 직접 구독하지 않습니다.

로그는 서비스로 실행 중이면 `onboard.log`에서, 수동 실행(`ros2 launch`)이면 각 노드가 남기는 `~/.ros/log/*.log`에서 읽습니다.

전환 순간의 로그를 함께 보려면 다른 터미널에서 실행하세요.

```bash
onboard_log
```

```
[auto_record_node] 녹화 시작 (armed) → flight_20260801_190312  [여유 392.9GB]
[auto_record_node] disarm 감지 — 10.0초 후 녹화 종료
[auto_record_node] 녹화 종료 (disarmed) — flight_20260801_190312 [67.3초, 여유 392.1GB]
```

원격 GUI가 필요하면 Foxglove Studio를 쓸 수 있습니다. mavros의 BEST_EFFORT QoS 문제 없이 모든 토픽을 볼 수 있습니다.

```bash
sudo apt install -y ros-humble-foxglove-bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

윈도우 PC의 Foxglove Studio에서 `ws://<모듈IP>:8765`로 접속합니다.

### 개발 모드와 운용 모드

서비스 등록과 부팅 자동 실행을 분리해 두었습니다. **개발 중에는 부팅 자동 실행을 켜지 마세요.**

```bash
bash scripts/install_service.sh           # 등록만 — 부팅 자동실행 OFF
bash scripts/install_service.sh enable    # 부팅 자동실행 ON (실기체 운용)
bash scripts/install_service.sh disable   # 다시 개발 모드로
bash scripts/install_service.sh status    # 현재 모드 확인 (= service_status)
bash scripts/install_service.sh remove    # 완전 제거
```

| 모드 | 부팅 시 | 수동 개발 작업 |
|---|---|---|
| 등록만 (기본) | 실행 안 함 | 자유롭게 가능 |
| `enable` | 자동 수집 시작 | 서비스 중지 후 진행 |

수동 제어 명령입니다.

```bash
sudo systemctl start   anomaly-sensor    # 지금 실행
sudo systemctl stop    anomaly-sensor    # 중지 (bag 정상 마감)
sudo systemctl status  anomaly-sensor
journalctl -u anomaly-sensor -f          # 실시간 로그
```

### 서비스와 수동 실행 충돌 방지

서비스가 이미 센서를 구동 중인데 `start_drone`을 실행하면 mavros가 중복 실행되고 시리얼 포트가 충돌합니다. 이를 막기 위해 `start_drone`이 서비스 상태를 먼저 확인하고, 실행 중이면 경고와 함께 중단합니다.

```
⚠️  'anomaly-sensor' 서비스가 이미 실행 중입니다
 먼저 서비스를 중지하세요:
   sudo systemctl stop anomaly-sensor
```

### 상태 확인

```bash
monitor_drone    # 서비스/노드/녹화/arm/센서 한 번에
onboard_log      # 실행 로그 실시간
```

`/auto_record/status` 토픽으로도 확인할 수 있습니다.

```bash
ros2 topic echo /auto_record/status --once
```
```json
{"recording": true, "armed": true, "bag": "flight_20260801_190312",
 "elapsed_s": 132.5, "free_gb": 47.31}
```

### 수동 제어

자동 녹화가 켜진 상태에서도 명령으로 직접 제어할 수 있습니다.

```bash
ros2 topic pub --once /auto_record/command std_msgs/String "{data: start}"
ros2 topic pub --once /auto_record/command std_msgs/String "{data: stop}"
```

### 자동 녹화 파라미터

```bash
ros2 launch drone_sensors drone_sensor_launch.py \
  use_auto_record:=true \
  save_dir:=/mnt/ssd/flight_logs \
  post_disarm_sec:=15.0 \
  max_bag_duration:=3000 \
  min_free_gb:=5.0
```

| 인자 | 기본값 | 설명 |
|---|---|---|
| `use_auto_record` | false | 자동 녹화 활성화 (온보드는 true) |
| `save_dir` | `~/anomaly_data` | 저장 경로 |
| `post_disarm_sec` | 10.0 | disarm 후 추가 녹화 시간 |
| `max_bag_duration` | 3000 | bag 분할 주기(초, 약 50분). 0이면 분할 안 함 |
| `min_free_gb` | 2.0 | 이보다 여유가 적으면 녹화 취소·중단 |
| `max_cache_bytes` | 8388608 | rosbag 내부 캐시(8MB). 전원 차단 시 손실 구간을 줄임. 0이면 기본값 사용 |
| `link_loss_stop_sec` | 120.0 | FC 링크가 이만큼 끊기면 녹화 종료 (그 전까지는 유지) |

### FC 전원 인가 순서와 USB 재열거링

온보드 모듈과 FC 전원이 동시에 들어가면, FC 부팅 과정에서 TELEM 라인 전압이 흔들려 **USB-TTL 젠더가 붙었다 떨어지기를 반복**할 수 있습니다. 이때 mavros가 먼저 포트를 열면 장치가 사라지면서 재연결에 실패합니다.

```
link[1000] reconnect failed: DeviceError:serial:open: No such file or directory
```

USB를 손으로 뽑았다 꽂으면 정상 동작하는 것이 이 현상의 특징입니다.

`start_onboard.sh`는 **연결된 시리얼 장치 목록이 `FC_STABLE_SEC`(기본 6초) 동안 변하지 않을 때까지** 기다린 뒤 launch를 실행합니다. 어느 장치가 FC인지는 launch가 데이터로 판별합니다.

```
시리얼 장치 안정화 대기 (FC 는 launch 가 자동 판별)
  최대 대기 90초 / 안정화 확인 6초
  장치 목록 변동 감지 — 안정화 카운터 초기화
장치 안정화 완료 (14초 경과)
```

기다린 뒤에도 FC가 늦게 붙거나 다시 끊기면 `fcu_manager_node`가 재탐지해 연결합니다.

기대하는 장치 수를 지정하면 그 수가 모일 때까지 기다립니다.

```bash
EXPECT_SERIAL=3 bash scripts/install_service.sh     # FC + THL100 + WCM6800
```

전원 인가가 더 불안정하면 값을 늘리세요.

```bash
FC_STABLE_SEC=15 WAIT_USB_SEC=120 bash scripts/install_service.sh
```

### FC 연결 감시 및 자동 복구

`fcu_manager_node`가 자동으로 처리합니다. 연결이 끊기면 **mavros만** 내리고 포트·baud·SYSID를 재탐지해 다시 띄우며, 센서 노드와 녹화는 끊기지 않습니다. 자세한 내용은 [장치 자동 구분과 FC 자동 연결](#장치-자동-구분과-fc-자동-연결)을 참고하세요.

```bash
fc_status          # 관리 상태 1회 (상시 확인은 monitor_drone)
```

예전 버전의 `watch_fcu`는 서비스 전체를 재시작했는데, 관리 노드와 겹치면 녹화까지 끊기므로 제거했습니다. `fc_status`는 상태만 보여줍니다.

### 비행별 기록 파일

녹화마다 bag 폴더 옆에 두 파일이 함께 생깁니다.

```
~/anomaly_data/
├── flight_20260921_190312/            rosbag
├── flight_20260921_190312_meta.json   녹화 조건·센서 상태
└── flight_20260921_190312_record.log  rosbag 출력 + 녹화 중 센서 이벤트
```

`_meta.json`은 녹화 시작과 종료 때 갱신됩니다.

| 항목 | 내용 |
|---|---|
| `preflight_ok`, `preflight` | 녹화 시작 시 센서 상태 |
| `sensor_events` | **녹화 중 센서 상태 변화** (예: 120초에 THL100 OK → STALE) |
| `sensors_ok_throughout` | 시작부터 종료까지 전 센서가 정상이었는지 |
| `duration_s`, `stop_reason`, `ended_at` | 녹화 길이와 종료 사유 |
| `verify` | **착륙 후 자동 검증 결과** — 드론 판정, 외부센서 상태 (`verify_bag`과 같은 기준) |
| `software` | 수집에 쓴 코드 커밋과 로컬 수정 여부. 모듈마다 버전이 다를 때 데이터셋 구분 기준 |

서비스와 노드는 살아 있는데 데이터만 멈춘 경우도 `sensor_events`에 남으므로, 비행 후 이 파일만 보면 해당 bag을 학습에 쓸 수 있는지 판단할 수 있습니다.

### 자원 사용

온보드 모듈에서 불필요한 부하를 줄이기 위해 다음을 적용했습니다.

| 항목 | 내용 |
|---|---|
| 상태 감시 | ReSpeaker 생존 확인을 12KB 오디오 대신 4바이트 `energy`로 (같은 콜백에서 발행되므로 오디오 경로를 동일하게 확인). **187KB/s 절감** |
| 분석기 타입 조회 | 메시지마다 하던 메시지 타입 조회를 토픽 타입별 1회로 캐시 |
| 마이크 오디오 | PCM을 바이트 그대로 발행 (리스트로 바꾸면 rclpy가 12,288개 원소를 하나씩 검사) |
| DoA/VAD 폴링 | 기본 25Hz. 틱마다 USB 제어 전송이 2회 일어나며, 50Hz로 두면 장치가 따라오지 못해 실측이 31Hz로 흔들렸습니다 |
| 모니터 | 저주기 토픽 4개만 구독. 센서 Hz는 `sensor_health_node`가 계산한 값을 받아 씀 |
| 장치 탐지 | MAVLink 조기 판정을 0.3초 간격으로만 수행 (매번 버퍼 전체를 다시 파싱하면 2.5초 탐색 중 0.6초를 파싱에 소모) |
| 분석기 | rosbag 메시지를 커서로 한 건씩 처리. 한 번에 읽으면 오디오 포함 bag에서 수백 MB가 메모리에 올라감 |

### 로그 관리

`~/anomaly_data/onboard.log`에 서비스 출력이 계속 append됩니다. 장기 운용 시 수백 MB까지 커져 `grep`이 바이너리 파일로 인식하기도 하므로, 서비스 시작 시 크기를 확인해 50MB를 넘으면 한 세대(`onboard.log.1`)만 남기고 새로 시작합니다.

```bash
LOG_MAX_MB=100 bash scripts/install_service.sh    # 임계값 변경
```

로그를 수동으로 비우려면 파일이 사용자 소유여야 합니다.

```bash
> ~/anomaly_data/onboard.log        # sudo 없이 동작해야 정상
```

`허가 거부`가 나면 소유권을 확인하세요. `sudo > file`은 리다이렉션을 현재 셸이 처리하므로 동작하지 않습니다.

```bash
sudo chown $USER:$USER ~/anomaly_data/onboard.log
sudo truncate -s 0 ~/anomaly_data/onboard.log
```

### 드론과 전원을 공유하는 경우

전원을 드론과 함께 쓰면 **전원 인가 즉시 켜지고, 리눅스 종료 절차 없이 전원이 내려갑니다.** 비행 중 차단은 추락을 뜻하므로 상정하지 않고, **착륙·녹화 종료 후 전원을 내리는 시점**을 기준으로 설계했습니다.

#### 언제 전원을 내려도 되는가

disarm 직후가 아닙니다. 착륙 후에도 `post_disarm_sec`(기본 10초) 동안 녹화가 이어지고, 그 뒤 bag 마감 → 메타·로그 기록 → 디스크 기록 순서를 거칩니다. **이 과정이 끝나기 전에 전원을 내리면 그 비행의 bag이 마감되지 못합니다.**

끝난 시점은 두 곳에서 확인할 수 있습니다.

```
monitor_drone  →  저장 완료 — 전원 차단 가능
onboard.log    →  저장 완료 — 지금부터 전원을 내려도 안전합니다 (flight_...)
```

녹화 중에는 모니터에 `⚠ 녹화 중 — 전원을 내리지 마세요`가 표시됩니다. 실무적으로는 **착륙 후 약 15초 뒤**에 내리시면 됩니다.

#### 기록 순서

판정에 쓰는 `_meta.json`과 `_record.log`도 아직 메모리에만 있을 수 있으므로, 기록 순서를 다음과 같이 맞췄습니다.

```
rosbag 마감(SIGINT) → 메타 기록(fsync) → 녹화 로그 기록(fsync) → sync
```

메타는 임시 파일에 쓴 뒤 이름을 바꾸고 디렉터리까지 `fsync`하므로, 쓰는 도중 전원이 끊겨도 이전 내용이 깨지지 않습니다.

#### 그 밖에 확인할 것

| 항목 | 내용 |
|---|---|
| **시각 보존** | RTC가 없으면 다음 부팅 때 시계가 과거로 돌아가 bag 이름·타임스탬프가 뒤엉킵니다. `setup_onboard_env.sh`가 RTC를 확인하고, 없으면 `fake-hwclock`을 설치해 **1시간 주기로 저장**합니다 (기본 설정은 정상 종료 때만 저장해 강제 차단에는 소용없음) |
| **부팅 시 파일시스템 검사** | 종료 절차 없이 내리는 일이 반복되면 부팅 때 검사가 걸릴 수 있습니다. 검사가 사용자 입력을 기다리면 그날 수집을 못 하므로, 커널 파라미터에 `fsck.mode=auto fsck.repair=yes`를 넣어 자동 복구되게 해두는 편이 안전합니다 |
| **준비 완료 시점** | 전원 인가 후 수집 준비까지 약 40~60초입니다. 준비된 순간이 로그에 `수집 준비 완료 — 전 센서 정상 (노드 시작 후 N초)`로 남습니다. 이륙 전 `monitor_drone --once`로 센서 4개가 `● 정상`인지 확인하세요 |
| **마감 실패 감지** | 그래도 마감되지 못한 bag이 생기면 `verify_bag`이 **"마감되지 않음(전원 차단 추정)"** 으로 표시하고 `ros2 bag reindex` 복구를 안내합니다 |
| 디스크 여유 | 매 전원 인가마다 자동 녹화되므로 쌓입니다. `min_free_gb`(기본 2GB) 밑이면 녹화가 중단되니 `verify_bag --all`로 점검 후 정리하세요 |

전원이 레귤레이터로 안정 공급되므로 rosbag 캐시는 8MB(약 25초분), `sync`는 15초 주기로 두었습니다. 추락이나 전압 이상 같은 예외적인 경우에만 그만큼이 손실됩니다.

### 전원 차단 대비

비행 중 갑작스러운 전원 차단은 완전히 막을 수 없지만, 손실을 최소화하도록 세 가지 장치를 두었습니다.

| 장치 | 효과 |
|---|---|
| `max_bag_duration` 3000초 분할 | 손상 시 마지막 구간만 영향, 이전 파일은 온전 |
| rosbag 캐시 8MB | 기본값(100MB)은 오디오 포함 시 **5분치가 메모리에만** 남습니다. 8MB면 약 25초분으로 줄어듭니다 (추락·전압 이상 대비) |
| 15초 주기 `sync` | 녹화 중 OS 디스크 캐시를 주기적으로 실제 기록 |
| systemd `KillSignal=SIGINT` | 정상 종료 시 rosbag이 파일을 올바르게 마감 |

`systemctl stop`이나 정상 종료(shutdown) 경로로는 bag이 손상되지 않습니다. 배터리를 갑자기 분리하는 상황만 위험하며, 이때도 분할된 이전 구간은 온전합니다.

---

## UART 센서 발행 구조

UART 수신과 토픽 발행을 분리하여 안정적인 주기를 보장합니다.

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
| 발행 타임스탬프 | `self.get_clock().now()` | ROS 시간 체계 일관성 |
| 경과시간 측정 (stale, drain) | `time.monotonic()` | NTP 시각 점프에 영향받지 않음 |

**일시적 오류와 실제 단선 구분**

pyserial은 데이터가 잠시 없을 때도 아래 예외를 던질 수 있습니다.

```
device reports readiness to read but returned no data
(device disconnected or multiple access on port?)
```

메시지와 달리 장치가 빠진 것이 아닌 경우가 많습니다. 이를 재연결로 처리하면 포트를 닫았다 여는 동작이 반복되어 오히려 수신율이 떨어지고 USB 재열거링을 유발할 수 있습니다.

수신 루프는 `in_waiting`이 0이면 블로킹하지 않고 짧게 대기 후 재확인하며, 위와 같은 일시적 오류는 포트를 유지한 채 재시도합니다. 진단 로그의 `transient` 항목으로 발생 횟수를 확인할 수 있습니다.

```
THL100 진단 [30s] rx=28 (0.93Hz) ok=28 fail=0 seq_gap=0 | 누적 rx=419 reconnect=1 transient=0
```

`reconnect`가 계속 늘어나면 실제 하드웨어 문제(전력·케이블)를 의심해야 합니다.

**기타**
- USB 분리 시 자동 재연결
- 스트림 버퍼 파서 — 나뉘어 온 패킷, 붙어 온 패킷, 쓰레기 데이터 모두 처리
- 30초마다 구간 기준 진단 로그

```
THL100 진단 [30s] rx=28 (0.93Hz) ok=28 fail=0 seq_gap=0 | 누적 rx=419 reconnect=1
WCM6800 진단 [30s] rx=92 (3.07Hz) ok=92 fail=0 | 누적 rx=1387 reconnect=1
```

### 발행 토픽

- `/thl100/data`, `/wcm6800/data` — 지정 주기, JSON (수신 시각, `age_ms` 포함)
- `/thl100/raw`, `/wcm6800/raw` — 수신 즉시 원본 패킷
- 개별 Float32 토픽 병행 발행 (하위 호환)

---

## 녹화 토픽 (46개)

### MAVROS
| 분류 | 토픽 |
|---|---|
| IMU / 자세 / 진동 | `/mavros/imu/data`, `/imu/data_raw`, `/imu/mag`, `/vibration/raw/vibration` |
| RC 입력 / 모터 출력 | `/mavros/rc/in`, `/rc/out` |
| 제어 목표값 | `/mavros/setpoint_raw/target_attitude`, `/target_local` |
| 로컬 위치·속도·가속도 | `/mavros/local_position/pose`, `/velocity_local`, `/accel` |
| GPS / 고도 | `/mavros/global_position/raw/fix`, `/raw/gps_vel`, `/raw/satellites`, `/global`, `/rel_alt`, `/mavros/gpsstatus/gps1/raw`, `/mavros/altitude` |
| 전력 / ESC | `/mavros/battery`, `/battery2`, `/esc_telemetry/telemetry`, `/esc_status/status` |
| 기체 상태 | `/mavros/vfr_hud`, `/state`, `/extended_state`, `/sys_status`, `/statustext/recv`, `/status_event`, `/timesync_status` |
| 항법 / 환경 | `/mavros/nav_controller_output/output`, `/wind_estimation` |
| 미션 / 지오펜스 | `/mavros/mission/waypoints`, `/mission/reached`, `/rallypoint/rallypoints`, `/geofence/fences` |

### 센서 및 부가
| 분류 | 토픽 |
|---|---|
| THL100 | `/thl100/data`, `/thl100/raw` |
| WCM6800 | `/wcm6800/data`, `/wcm6800/raw` |
| ReSpeaker | `/respeaker/doa`, `/vad`, `/energy`, `/audio` |
| 라벨 / 메타 | `/anomaly/label`, `/test/metadata` |
| 진단 | `/diagnostics` |

`/respeaker/audio`는 16kHz 6채널 int16 PCM 원본입니다. 데이터량이 크므로 저장 용량을 확인하세요.

| 항목 | 값 |
|---|---|
| 대역 | 약 187 KB/s |
| 시간당 | 약 0.64 GB |
| 3000초 분할 파일 | 약 0.59 GB |

### 라벨 및 메타데이터 발행

`/anomaly/label`과 `/test/metadata`는 이 패키지가 발행하지 않습니다. 실험 중 외부에서 발행하면 함께 녹화되어 학습 라벨로 활용할 수 있습니다.

```bash
ros2 topic pub -r 1 /anomaly/label std_msgs/Int32 "{data: 1}"

ros2 topic pub --once /test/metadata std_msgs/String \
  "{data: '{\"dronetype\":\"quad_x\",\"flightNo\":3,\"abnormal_type\":\"motor_degrade\"}'}"
```

---

## 데이터 녹화 (수동)

```bash
record_drone 60     # 60초
record_drone         # 무제한 (Ctrl+C)
```

`~/anomaly_data/anomaly_data_YYYYMMDD_HHMMSS/` 에 저장됩니다. 자동 녹화 파일은 `flight_` 접두어로 구분됩니다.

```bash
ANOMALY_DATA=/mnt/ssd/flight_logs record_drone 60    # 저장 경로 변경
```

---

## 시간 동기화

`start_drone`은 실행 전 시간 동기화 상태를 확인합니다.

- 인터넷 연결 + NTP 미동기화 → 자동 동기화 시도
- 인터넷 없음 → 경고 출력 후 계속 진행

야외 오프라인 비행 시에는 **비행 전 한 번 인터넷에 연결**하여 시간을 맞춰두는 것을 권장합니다.

```bash
timedatectl status
```

---

## 데이터 분석

```bash
analyze_drone ~/anomaly_data/flight_20260803_111347
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
| 2 | UART JSON 내부 수신 시각 |
| 3 | rosbag 기록 시각 (Header 없는 메시지) |

개별 CSV에는 네 값이 보존됩니다.

- `source_time_ns` — 데이터 발생 시각
- `bag_time_ns` — rosbag 기록 시각
- `transport_delay_ms` — 두 값의 차이
- `stamp_source` — 어떤 시각을 사용했는지 (`header` / `uart` / `bag` / `bag(skew)`)

**header.stamp 신뢰성 검증**

GPS fix가 없으면 ArduPilot이 부팅 후 경과시간을 타임스탬프로 사용해, `header.stamp`가 시스템 시각과 수 시간까지 어긋날 수 있습니다. 분석기는 차이가 `MAX_STAMP_SKEW_SEC`(기본 5초)를 넘으면 `bag_time`을 사용하며 경고를 출력합니다.

### 좌표계

MAVROS는 FC의 NED 데이터를 **ENU**로 변환해 발행합니다.

| 컬럼 | 좌표계 |
|---|---|
| `LocalENU_X/Y/Z`, `LocalENU_VX/VY/VZ` | ENU (MAVROS 원본) |
| `LocalNED_N/E/D`, `LocalNED_VN/VE/VD` | NED 변환값 (N=ENU_Y, E=ENU_X, D=−ENU_Z) |
| `GPS_CourseAngle` | 항공 course (북 0°, 시계방향) |

### 제어 계층과 추종 오차

자세 관련 데이터는 세 계층에서 나옵니다.

```
NAV_CONTROLLER_OUTPUT (#62)   항법 계층 목표      Nav_Roll / Nav_Pitch
        ↓
ATTITUDE_TARGET (#83)         자세 제어 목표      ATT_DesRoll / ATT_DesPitch / ATT_DesYaw
        ↓
ATTITUDE (#30)                실제 기체 자세      ATT_Roll / ATT_Pitch / ATT_Yaw
```

분석 시 오차 컬럼이 자동 생성됩니다.

| 컬럼 | 계산 | 의미 |
|---|---|---|
| `Err_Roll/Pitch/Yaw` | `ATT_Des* − ATT_*` | 자세 제어 추종 오차 |
| `ErrNav_Roll/Pitch` | `Nav_* − ATT_*` | 항법 목표 대비 오차 |
| `Err_RateR/P/Y` | `RATE_*Des − RATE_*` | 각속도 추종 오차 |
| `Err_PosX/Y/Z`, `Err_VelX/Y/Z` | `Des_ENU_* − LocalENU_*` | 위치·속도 추종 오차 |
| `Err_AttMag` | `√(Err_Roll² + Err_Pitch²)` | 자세 오차 크기 |

Yaw 오차는 `-180~180`으로 정규화됩니다.

#### type_mask 확인

`setpoint_raw/target_*`의 `type_mask`는 어떤 필드를 무시해야 하는지 나타냅니다.

| 컬럼 | 의미 |
|---|---|
| `ATT_Des_AttValid` | 자세 유효 여부 (bit7) |
| `ATT_Des_RateValid` | body rate 유효 여부 (bit0~2) |
| `Des_PosValid` | 위치 유효 여부 (bit0~2) |
| `Des_VelValid` | 속도 유효 여부 (bit3~5) |

값이 0인 구간의 목표값은 분석에서 제외해야 합니다.

### 미션 데이터

`/mavros/mission/waypoints`는 미션을 업로드하거나 변경할 때만 발행되는 latched 성격입니다. 녹화 시작 시점에 한 번 기록되고, 이후 변경될 때만 추가됩니다.

웨이포인트 전체를 CSV 컬럼으로 펼칠 수 없으므로 요약만 기록합니다.

| 컬럼 | 내용 |
|---|---|
| `Mission_Count` | 총 웨이포인트 수 |
| `Mission_Current` | 현재 목표 seq |
| `Mission_TgtLat/Lon/Alt` | 현재 목표 웨이포인트 좌표 |
| `Mission_TgtCmd` | 현재 목표의 MAVLink 명령 코드 |
| `Mission_ReachedSeq` | 도달한 웨이포인트 seq (이벤트) |
| `Rally_Count`, `Fence_Count` | 랠리 포인트·지오펜스 개수 |

전체 경로는 개별 CSV(`mavros_mission_waypoints.csv`)의 `Mission_Waypoints` 컬럼에 JSON으로 보존됩니다.

```json
[{"seq": 0, "cmd": 22, "frame": 3, "lat": 37.1, "lon": 127.1, "alt": 10.0,
  "p1": 0.0, "p2": 0.0, "p3": 0.0, "p4": 0.0, "autocont": true}]
```

명령 분포는 `Mission_Cmds`에 `16x2,21x1,22x1` 형태로 요약됩니다(웨이포인트 2개, 착륙 1개, 이륙 1개). 주요 MAVLink 명령 코드입니다.

| 코드 | 의미 |
|---|---|
| 16 | `NAV_WAYPOINT` |
| 21 | `NAV_LAND` |
| 22 | `NAV_TAKEOFF` |
| 20 | `NAV_RETURN_TO_LAUNCH` |
| 17 | `NAV_LOITER_UNLIM` |

`Mission_Waypoints`와 `Mission_Cmds`는 문자열이라 merged CSV에서는 제외됩니다.

### 마이크 원본 오디오

`/respeaker/audio`의 PCM 파형은 CSV에 담을 수 없어 프레임별 요약 통계만 기록합니다.

| 컬럼 | 내용 |
|---|---|
| `MICRaw_RMS` | ch0 프레임 RMS |
| `MICRaw_Peak` | 최대 진폭 |
| `MICRaw_ClipPct` | 클리핑 샘플 비율(%) |
| `MICRaw_Samples` | 프레임당 샘플 수 |

파형 자체가 필요하면 WAV로 추출하세요.

```bash
extract_audio ~/anomaly_data/flight_20260803_111347           # ch0
extract_audio ~/anomaly_data/flight_20260803_111347 --all     # 6채널 전부
extract_audio ~/anomaly_data/flight_20260803_111347 --ch 1    # 특정 채널
```

### 녹화 검증 — `verify_bag`

비행 후 녹화가 제대로 됐는지 확인합니다. **드론 데이터로 판정**하고, 외부센서는 옆에 수집 상태를 표시합니다.

녹화가 끝나면 같은 판정이 **자동으로 실행**되어 `_meta.json`의 `verify`에 기록되고, `monitor_drone`의 최근 녹화 목록에 바로 표시됩니다(`드론 정상 · THL100 확인`). 로그에도 `비행 검증 — 드론 정상 / 외부센서 전부 정상` 형태로 남습니다. 수동 실행은 과거 bag을 다시 보거나 기준을 바꿨을 때 사용합니다.

```bash
verify_bag                          # 가장 최근 bag
verify_bag flight_20260921_190312   # 특정 bag
verify_bag --all                    # 전체 요약표
verify_bag --all --csv list.csv     # 요약표를 CSV 로 (학습 데이터 선별용)
```

| 구분 | 대상 | 결과 |
|---|---|---|
| **드론 (판정)** | 자세/각속도, 가속도/자이로, 모터 PWM, 진동, 배터리, 상태 | 정상 / 주의 / 불량 |
| **외부센서 (표기)** | ReSpeaker, THL100, WCM6800 | 정상 / 부분 / 없음 |

| 드론 판정 | 조건 |
|---|---|
| 정상 | 모든 필수 토픽이 기대 주기의 70% 이상, 끊김 없음 |
| 주의 | 짧은 끊김이 있거나 주기가 낮음 (FC의 SR 설정이 권장값과 다른 경우 포함) |
| 불량 | 필수 토픽 없음, 5초 이상 끊김, 분할 파일 누락 |

외부센서는 "부분"이면 데이터는 있지만 공백이 있다는 뜻이며, 첫 공백 위치와 길이를 함께 보여줍니다. 녹화 시작 후 센서가 늦게 붙은 경우도 공백으로 잡힙니다. setpoint·ESC·미션처럼 모드나 장비에 따라 안 나오는 토픽은 판정에서 제외합니다.

`_meta.json`의 프리플라이트 결과와 비행 중 센서 이벤트, `_record.log`의 경고도 함께 표시합니다.

```
flight_20260921_190312   길이 412초   비행 385초 (arm 1회)
────────────────────────────────────────────────────────────────
  드론 데이터   ✅ 정상
     자세/각속도      49.8Hz  (기대   50)
     ...
  외부센서
     ✅ ReSpeaker  정상  15.6Hz
     ⚠  THL100     부분  1.0Hz  공백 1회 (첫 공백 120초부터 38초)
     ✅ WCM6800    정상  10.0Hz
  비행 중 이벤트 2건: 120초 THL100 OK→STALE, 158초 THL100 STALE→OK
────────────────────────────────────────────────────────────────
판정: 드론 정상
   외부센서: THL100 확인 필요
```

**결과 재사용** — bag 하나 검증에 50분 비행 기준 약 4초가 걸려, 비행이 100개면 `--all`이 6분 정도 됩니다. 그래서 bag 파일(크기·수정 시각)과 판정 기준이 그대로면 이전 결과를 씁니다(`~/anomaly_data/.verify_cache.json`). 착륙 후 자동 검증 결과도 여기에 저장되므로 보통은 즉시 끝납니다. 새 비행만 새로 검증하고, 판정 기준을 바꾸면 자동으로 전부 다시 검증합니다. 강제로 다시 하려면 `verify_bag --all --recheck`.

기대 주기와 허용 공백은 `scripts/verify_bag.py` 상단의 `DRONE_TOPICS`, `SENSOR_TOPICS`에서 조정할 수 있습니다. 드론 판정이 불량이면 종료 코드 2를 반환하므로 스크립트에서도 쓸 수 있습니다.

### 분할 bag 처리

`max_bag_duration`(기본 3000초)에 도달하면 rosbag2가 파일을 분할합니다.

```
flight_20260803_111347/
├── metadata.yaml
├── flight_20260803_111347_0.db3
├── flight_20260803_111347_1.db3
└── flight_20260803_111347_2.db3
```

분석기는 `metadata.yaml`의 순서를 따라 **모든 파일을 읽습니다.** metadata가 없으면 파일명의 숫자 접미사로 정렬합니다(사전순 정렬 시 `_10`이 `_2`보다 앞서는 문제 방지).

> **모델 학습용 데이터**
>
> `_merged_10hz.csv`는 여러 토픽을 한 표로 보기 위한 **요약본**입니다.
> 10Hz 격자에 맞추느라 원본 주기(50Hz 등)가 다운샘플링되므로,
> AI 모델 학습에는 `~/anomaly_data/<bag>/` 의 **원본 rosbag** 또는
> 토픽별 개별 CSV(`<bag>_csv/`)를 사용하세요.

**10Hz 정렬 방식**

- 기준 시간축: 100ms 간격
- `merge_asof(direction='backward')` — 미래 데이터 사용 금지
- 센서 그룹별 `*_age_ms` — 해당 값이 얼마나 오래된 것인지
- `*_stale` 플래그 — 허용 age 초과 여부 (1/0)
- 허용 age 초과 시 값은 `NaN` 처리

타임존은 `scripts/analyze_bag.py` 상단 `LOCAL_TZ`로 변경 가능합니다.

---

## 테스트

```bash
# 파서 / 좌표변환 / 추종오차 단위 테스트 (ROS2 없이 실행 가능)
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
| VIBE X/Y/Z | 지원 | `SR2_EXTRA3` + 커스텀 pluginlist (적용됨) |

일부 토픽은 조건부로만 발행됩니다.

| 토픽 | 발행 조건 |
|---|---|
| `local_position/*` | GPS fix로 EKF 수렴 필요 |
| `setpoint_raw/target_*` | arm 상태에서 제어 루프 동작 시 |
| `esc_telemetry/*` | ESC 텔레메트리 지원 ESC 필요 |
| `battery2` | 배터리 모니터 2번 설정 시 |
| `anomaly/label`, `test/metadata` | 외부 발행 필요 |
| `mission/waypoints` | 미션 업로드 시 (latched, 변경 시에만 재발행) |
| `mission/reached` | 웨이포인트 도달 시 |

---

## 정상 동작 확인

실행 후 30초 뒤 진단 로그가 아래와 같으면 정상입니다.

```
THL100 진단 [30s] rx=28 (0.93Hz) ok=28 fail=0 seq_gap=0
WCM6800 진단 [30s] rx=92 (3.07Hz) ok=92 fail=0
```

실내 환경에서 아래 mavros 메시지는 정상입니다.

| 메시지 | 의미 |
|---|---|
| `GP: No GPS fix` | 실내라 GPS 미수신. 야외에서 해소 |
| `TM: Wrong FCU time` | GPS 없어 FC 시각 동기화 불가 |
| `PreArm: Check mag field` | 나침반 보정 필요. 데이터 수집에는 무관 |
| `Plugin xxx ignored` | 의도적으로 차단한 플러그인 (vibration/altitude가 아니면 정상) |

---

## 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| 젠더 교체 후 `No such file or directory` | `by-id`는 젠더 개체의 시리얼 번호 기반이라 교체 시 경로가 바뀜 | 자동 탐색이 처리함(적용됨). `detect_serial`로 확인 |
| 같은 모델 젠더 2개를 구분 못 함 | VID:PID가 동일 | 데이터 시그니처로 판별(적용됨) |
| USB 재삽입 후 FC만 재연결 안 됨 (`reconnect failed: No such file`) | mavros가 시작 시 경로를 고정 → 번호가 바뀌면 복구 못 함 | `fcu_manager_node`가 재탐지 후 재기동 (적용됨). `fc_status`로 확인 |
| FC가 HEARTBEAT를 보내는데 `connected: false` | FC의 SYSID가 1이 아님 (예: 2) | HEARTBEAT로 SYSID 자동 탐지 (적용됨). `detect_serial`로 확인 |
| `detected remote address 191.239` 같은 이상한 주소 | 잘못된 baud의 잡음을 MAVLink로 오인 | CRC 검증으로 차단 (적용됨) |
| 비행제어기 여러 대 경고 | 같은 링크에 다른 기체가 중계됨 | `tgt_system:=N`으로 대상 지정 |
| 첫 연결인데 `재시작 1회` | 센서 노드가 추측한 폴백 경로로 FC 포트를 먼저 열어 HEARTBEAT를 나눠 가짐 | FC 포트 선점 표시 + 센서 폴백 `auto` (적용됨). 모니터의 '마지막 재시작 사유' 확인 |
| 모니터 윗부분이 반복돼 보임 | 창보다 화면이 길어 넘친 줄이 스크롤백에 쌓임 | 별도 화면 사용 + 크기 맞춤 (적용됨). 창을 40줄 이상으로 |
| `git clone` 시 `이미 있고 빈 디렉터리가 아닙니다` | 같은 이름의 폴더가 이미 존재 | 기존 것을 지우고 clone 하거나 `git pull`로 갱신 ([2단계](#2단계--저장소-클론-및-설치) 참고) |
| mavros 실행 실패 (`libdiagnostic_updater.so`) | diagnostic 패키지 미설치 | `sudo apt install ros-humble-diagnostic-updater ros-humble-diagnostic-msgs` (install.sh 반영) |
| `lsusb`엔 CH340이 보이는데 `/dev/ttyUSB*`가 없음 | `brltty`가 CH340을 점자 장치로 오인 | `bash scripts/setup_onboard_env.sh` 후 USB 재삽입 |
| 진동 토픽이 목록에 없음 (`Unknown topic`) | mavros 기본 pluginlist가 `vibration` 차단 | 커스텀 `config/apm_pluginlists.yaml` 사용 (적용됨) |
| 진동 토픽은 있으나 데이터 없음 | `SR2_EXTRA3` 미설정 | Mission Planner에서 `SR2_EXTRA3=20` |
| 연결은 되는데 IMU 토픽이 안 옴 | 포트에 맞는 SR 파라미터 미설정 | TELEM2면 `SR2_*`, USB면 `SR0_*` |
| `VER: broadcast request timeout` 반복 | baud 불일치 또는 TELEM 포트 비활성 | baud 확인, `SERIAL2_PROTOCOL`/`BAUD` 점검 |
| `serial:open: No such file or directory` | `fcu_url` 경로 불일치 | `detect_serial`로 확인 (평소에는 자동 탐지) |
| 부팅 후 mavros만 연결 실패, USB 재삽입하면 정상 | FC 전원 인가 중 젠더 재열거링 | `FC_STABLE_SEC` 안정화 대기 (적용됨) |
| 서비스는 active인데 `monitor_drone`에 아무것도 안 보임 | 셸의 `ROS_DOMAIN_ID` 불일치 | `export ROS_DOMAIN_ID=0` (자동 등록됨) |
| `ros2 node list`가 0개, 모니터가 비어 보임 | `ros2 daemon` 크래시 | 자동 복구됨. **녹화에는 영향 없음** |
| 시리얼 `Permission denied` | 장치 권한 미설정 | `fix_permissions` (재로그인 불필요) |
| 수동 실행 시 mavros 크래시 | 서비스가 이미 구동 중 | `sudo systemctl stop anomaly-sensor` |
| `onboard.log`에 `허가 거부` | systemd가 root 소유로 생성 | `setup_onboard_env.sh`가 자동 처리 |
| `grep`이 로그를 "바이너리 파일"로 인식 | 로그 과대 | 50MB 초과 시 자동 로테이션. `grep -a` 사용 가능 |
| `colcon build` File exists | 이전 빌드 잔여 | install.sh가 자동 정리 |
| `ros2 run` 패키지 못 찾음 | resource 마커 누락 | `bash fix_packaging.sh` 후 재빌드 |
| `KeyError: 'launch'` | `launch/__init__.py` 존재 | 자동 제거됨 |
| matplotlib NumPy 오류 | NumPy 2.x 비호환 | `pip install "numpy<2"` (적용됨) |
| 분할 bag 뒷부분 누락 | 첫 `.db3`만 읽던 구버전 | `metadata.yaml` 기준 전체 읽기 (적용됨) |
| 분석 시 duration이 수 시간 | GPS 없어 `header.stamp` 이상 | `bag_time` 자동 대체 (적용됨) |
| `datetime` 9시간 차이 | UTC → KST 변환 누락 | `LOCAL_TZ` 기준 변환 (적용됨) |
| `monitor_drone`에서 저주기 토픽이 `--` | `topic hz` 타임아웃 부족 | 8초로 상향 (적용됨) |
| FC 연결 끊김 (`No such device`) | USB 분리/FC 재부팅 | 재연결 후 `stop_drone` → `start_drone` |
| UART 센서가 `readiness to read but returned no data` 반복, 수신율 하락 | `read(in_waiting or 1)`이 데이터 없을 때 블로킹하다 예외 발생 → 불필요한 재연결 | `in_waiting`이 0이면 짧게 대기 후 재확인하도록 수정(적용됨). 일시적 오류는 포트를 닫지 않음 |
| `dmesg`에 `USB disconnect` 반복 | 허브 전력 부족, 케이블 접촉, TELEM2 VCC 연결 | 젠더를 본체 포트에 직접 연결해 확인. TELEM2 VCC(1번 핀)는 **연결 금지** |
