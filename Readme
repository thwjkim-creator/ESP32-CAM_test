# ESP32-CAM + VEML7700 전동 블라인드 센싱 모듈

전동 블라인드 자동 제어 시스템의 **센싱 모듈**입니다.
ESP32-CAM으로 실내 사진을 촬영하고, VEML7700 조도 센서로 주변 밝기를 측정하여 서버로 전송합니다.
서버는 수신된 데이터를 기반으로 블라인드 모터 모듈에 제어 명령을 내립니다.

## 시스템 구조

```
┌──────────────────┐         HTTP POST (JPEG)        ┌────────────┐
│  ESP32-CAM       │ ──────────────────────────────→  │            │
│  + VEML7700      │         MQTT (조도 JSON)         │   Server   │
│  (센싱 모듈)     │ ──────────────────────────────→  │            │
└──────────────────┘                                  │            │
                                                      │            │  MQTT (제어 명령)
┌──────────────────┐                                  │            │ ──────────────→  모터 모듈 ×5
│  Mosquitto       │◄────────────────────────────────►│            │
│  (MQTT Broker)   │                                  └────────────┘
└──────────────────┘
```

## 필요 하드웨어

| 부품 | 모델 | 비고 |
|------|------|------|
| 메인 보드 | AI-Thinker ESP32-CAM | OV2640 카메라 내장, PSRAM 탑재 |
| 조도 센서 | DFRobot SEN0228 (VEML7700) | I2C 통신, Auto-Gain 지원 |

### 배선

| SEN0228 핀 | ESP32-CAM GPIO | 비고 |
|-----------|---------------|------|
| SDA | GPIO 14 | I2C_NUM_1 |
| SCL | GPIO 15 | I2C_NUM_1 |
| VCC | 3.3V | |
| GND | GND | |

> 카메라 SCCB는 I2C_NUM_0 (GPIO 26/27)을 내부적으로 사용합니다.
> VEML7700은 I2C_NUM_1에 배치하여 버스 충돌을 방지합니다.

## 개발 환경

- **IDE**: VS Code + ESP-IDF Extension
- **ESP-IDF**: v5.5.2
- **프레임워크**: 순수 ESP-IDF (PlatformIO 아님)
- **Python**: 3.10+ (서버용)

## 프로젝트 구조

```
├── CMakeLists.txt              # 최상위 빌드 설정
├── sdkconfig.defaults          # ESP-IDF 기본 설정 (PSRAM, 카메라, SCCB 포트 등)
├── partitions.csv              # 커스텀 파티션 (factory 3MB)
├── main/
│   ├── CMakeLists.txt          # 컴포넌트 빌드 설정
│   ├── idf_component.yml       # esp32-camera 의존성
│   ├── main.c                  # Wi-Fi, MQTT, HTTP, FreeRTOS 태스크
│   ├── camera_drv.h / .c       # OV2640 카메라 래퍼
│   └── veml7700_drv.h / .c     # VEML7700 드라이버 (Auto-Gain)
└── server.py                   # 수신 서버 (HTTP + MQTT)
```

## 실행 전 설정 변경

### 1. ESP32 펌웨어 (`main/main.c`)

```c
// Wi-Fi 설정
#define WIFI_SSID              "YOUR_WIFI_SSID"      // ← Wi-Fi 이름
#define WIFI_PASS              "YOUR_WIFI_PASS"       // ← Wi-Fi 비밀번호

// 서버 주소
#define HTTP_POST_URL          "http://192.168.1.100:8080/upload"   // ← 서버 IP
#define MQTT_BROKER_URI        "mqtt://192.168.1.100:1883"          // ← MQTT 브로커 IP

// 측정 주기
#define TASK_PERIOD_MS         10000   // ← 카메라 + 센서 공통 주기 (ms)
```

### 2. 수신 서버 (`server.py`)

```python
HTTP_PORT   = 8080          # HTTP 수신 포트
MQTT_BROKER = "127.0.0.1"  # MQTT 브로커 주소 (같은 PC면 127.0.0.1)
MQTT_PORT   = 1883          # MQTT 브로커 포트
```

### 3. 외부 서버 사용 시

ESP32와 서버가 다른 네트워크에 있는 경우:

- `HTTP_POST_URL`과 `MQTT_BROKER_URI`를 서버의 **공인 IP 또는 도메인**으로 변경
- 서버 방화벽에서 **8080 (HTTP)**, **1883 (MQTT)** 포트 개방
- Mosquitto에 **인증 및 TLS** 설정 권장

## 실행 방법

### A. 로컬 테스트 (같은 Wi-Fi 내 PC에서 모두 실행)

#### Step 1. Mosquitto 설정 및 실행

`mosquitto.conf` 파일을 만들고 아래 내용 입력 (mosquitto.conf는 mosquitto 다운받은 폴더에 존재):

```conf
# ── mosquitto.conf (로컬 테스트용) ──
listener 1883 0.0.0.0
allow_anonymous true
```

> `allow_anonymous true` — 인증 없이 접속 허용 (로컬 테스트 전용)
> `listener 1883 0.0.0.0` — 같은 네트워크의 ESP32에서 접속 가능하도록 모든 인터페이스에서 수신

실행:

```bash (cmd창)
mosquitto -c mosquitto.conf -v
```

#### Step 2. 수신 서버 실행

```bash (cmd창)
pip install -r requirements.txt
python server.py
```

정상 실행 시 출력:

```
[HTTP] 서버 시작: http://localhost:8080
[HTTP] 갤러리:   http://localhost:8080/gallery
[MQTT] 구독 시작: sensor/veml7700/lux
```

#### Step 3. ESP32 설정 및 플래시

`main/main.c`에서 PC의 로컬 IP를 입력합니다:

```c
#define WIFI_SSID        "YOUR_WIFI_SSID"
#define WIFI_PASS        "YOUR_WIFI_PASS"
#define HTTP_POST_URL    "http://192.168.x.x:8080/upload"   // ← PC의 로컬 IP
#define MQTT_BROKER_URI  "mqtt://192.168.x.x:1883"          // ← 같은 IP
```

> PC의 로컬 IP 확인: `ipconfig` (Windows) / `ifconfig` (Mac/Linux)

빌드 및 플래시 (터미널):

```bash (cmd창)
idf.py set-target esp32
idf.py build
idf.py -p COMx flash monitor
```

빌드 및 플래시 (VS Code ESP-IDF Extension):

1. VS Code에서 프로젝트 폴더 열기
2. `Ctrl+Shift+P` → **ESP-IDF: Set Espressif Device Target** → `ESP32` 선택
3. `Ctrl+Shift+P` → **ESP-IDF: Build your Project** (또는 하단 상태바 🔨 클릭)
4. `Ctrl+Shift+P` → **ESP-IDF: Select Port to Use** → COM 포트 선택
5. `Ctrl+Shift+P` → **ESP-IDF: Flash your Project** (또는 하단 상태바 ⚡ 클릭)
6. `Ctrl+Shift+P` → **ESP-IDF: Monitor Device** (또는 하단 상태바 📺 클릭)

> 최초 빌드 시 `idf_component.yml`에 의해 esp32-camera가 자동 다운로드됩니다.

#### Step 4. 동작 확인

- **ESP32 시리얼 모니터**: JPEG 촬영 / Lux 측정 / HTTP·MQTT 전송 로그 확인
- **서버 콘솔**: `[HTTP] 사진 저장` / `[MQTT] 조도 수신` 로그 확인
- **갤러리 페이지**: 브라우저에서 `http://localhost:8080/gallery` 접속

---

### B. 외부 서버 배포 (ESP32와 서버가 다른 네트워크)

#### Step 1. Mosquitto 설정 및 실행

`mosquitto.conf` 파일:

```conf
# ── mosquitto.conf (외부 서버용) ──
listener 1883 0.0.0.0

# 인증 필수
allow_anonymous false
password_file /etc/mosquitto/passwd

# TLS 암호화 (권장)
# listener 8883 0.0.0.0
# certfile /etc/mosquitto/certs/server.crt
# keyfile /etc/mosquitto/certs/server.key
# cafile /etc/mosquitto/certs/ca.crt
```

사용자 계정 생성:

```bash
mosquitto_passwd -c /etc/mosquitto/passwd esp32user
# 비밀번호 입력 프롬프트가 나옵니다
```

실행:

```bash
mosquitto -c mosquitto.conf -v
```

#### Step 2. 서버 방화벽 설정

| 포트 | 용도 | 필수 |
|------|------|------|
| 8080 | HTTP (사진 수신) | O |
| 1883 | MQTT (조도 수신/모터 제어) | O |
| 8883 | MQTT over TLS | TLS 사용 시 |

#### Step 3. 수신 서버 실행

`server.py`의 브로커 주소를 확인합니다 (같은 서버에서 실행하면 `127.0.0.1` 유지):

```bash
pip install -r requirements.txt
python server.py
```

#### Step 4. ESP32 설정 및 플래시

`main/main.c`에서 서버의 공인 IP 또는 도메인을 입력합니다:

```c
#define HTTP_POST_URL    "http://your-server.com:8080/upload"
#define MQTT_BROKER_URI  "mqtt://your-server.com:1883"
```

MQTT 인증을 사용하는 경우, `main.c`의 MQTT 설정에 username/password를 추가해야 합니다:

```c
const esp_mqtt_client_config_t cfg = {
    .broker.address.uri = MQTT_BROKER_URI,
    .credentials = {
        .username = "esp32user",
        .authentication.password = "YOUR_PASSWORD",
    },
    // ... 기존 설정 유지
};
```

## 데이터 흐름

| 데이터 | 프로토콜 | 주기 | 엔드포인트 |
|--------|---------|------|-----------|
| JPEG 사진 | HTTP POST | 10초 | `/upload` (X-Timestamp 헤더 포함) |
| 조도 (lux) | MQTT QoS 1 | 10초 | `sensor/veml7700/lux` (JSON) |

### MQTT 메시지 형식

```json
{
  "lux": 523.45,
  "timestamp": "2026-03-24T14:01:26Z"
}
```

### MQTT 상태 토픽

| 토픽 | 메시지 | 조건 |
|------|--------|------|
| `sensor/veml7700/status` | `{"status":"online"}` | 연결 시 |
| `sensor/veml7700/status` | `{"status":"offline"}` | LWT (비정상 연결 해제 시) |

## VEML7700 Auto-Gain 알고리즘

센서 raw 값에 따라 Gain과 Integration Time을 자동 조절합니다.

| 조건 | 동작 |
|------|------|
| raw < 100 | Gain/IT 증가 (어두운 환경 대응) |
| raw > 10000 | Gain/IT 감소 (밝은 환경 대응) |
| lux > 1000 | Datasheet 비선형 보정 공식 적용 |

## 주의사항

- `sdkconfig.defaults` 변경 후에는 반드시 기존 `sdkconfig` 삭제 후 클린 빌드:
  ```bash
  del sdkconfig
  idf.py fullclean
  idf.py build
  ```
- VEML7700 init은 카메라 init보다 **반드시 먼저** 호출 (`app_main` 순서 참고)
- SCCB 포트는 `sdkconfig.defaults`에서 `CONFIG_SCCB_HARDWARE_I2C_PORT0=y`로 고정
- VEML7700 드라이버는 `driver/i2c_master.h` (신규 API) 사용 — legacy API는 esp32-camera와 충돌