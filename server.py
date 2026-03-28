"""
ESP32-CAM + VEML7700 수신 서버
- HTTP POST /upload  → JPEG 저장
- MQTT subscribe     → sensor/veml7700/lux JSON 수신
- GET /gallery       → 사진 + 조도 매칭 갤러리
- GET /images/...    → 이미지 서빙

실행: pip install paho-mqtt 후 python server.py
"""

from http.server import HTTPServer, BaseHTTPRequestHandler
from datetime import datetime
import threading
import json
import os

# ── pip install paho-mqtt 필요 ──
try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    print("[MQTT] paho-mqtt 미설치 — pip install paho-mqtt")
    print("[MQTT] MQTT 없이 HTTP만 동작합니다.")
    MQTT_AVAILABLE = False

# ══════════════════════════════════════════════
#  설정
# ══════════════════════════════════════════════
HTTP_PORT   = 8080
MQTT_BROKER = "127.0.0.1"
MQTT_PORT   = 1883
MQTT_TOPIC  = "sensor/veml7700/lux"

photo_log  = []   # [{ 'timestamp': str, 'file': str }, ...]
sensor_log = []   # [{ 'timestamp': str, 'lux': float }, ...]
log_lock   = threading.Lock()

# ══════════════════════════════════════════════
#  MQTT Subscriber (센서 데이터 수신)
# ══════════════════════════════════════════════
def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        client.subscribe(MQTT_TOPIC, qos=1)
        print(f"[MQTT] 구독 시작: {MQTT_TOPIC}")
    else:
        print(f"[MQTT] 연결 실패: rc={rc}")

def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        entry = {
            "timestamp": data.get("timestamp", ""),
            "lux":       data.get("lux", 0.0),
        }
        with log_lock:
            sensor_log.append(entry)
        print(f"[MQTT] 조도 수신: {entry['lux']} lux  ({entry['timestamp']})")
    except Exception as e:
        print(f"[MQTT] 파싱 오류: {e}")

def mqtt_thread():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_forever()
    except Exception as e:
        print(f"[MQTT] 브로커 연결 실패: {e}")
        print("[MQTT] MQTT 없이 HTTP만 동작합니다.")

# ══════════════════════════════════════════════
#  타임스탬프 매칭
# ══════════════════════════════════════════════
def parse_ts(ts_str):
    """ISO 8601 문자열 → datetime"""
    for fmt in ("%Y-%m-%dT%H:%M:%SZ", "%Y-%m-%dT%H:%M:%S"):
        try:
            return datetime.strptime(ts_str, fmt)
        except ValueError:
            continue
    return None

def find_closest_sensor(photo_ts_str, max_diff_sec=10):
    """사진 타임스탬프와 가장 가까운 센서 데이터 반환"""
    photo_dt = parse_ts(photo_ts_str)
    if not photo_dt:
        return None

    best = None
    best_diff = float("inf")
    for s in sensor_log:
        s_dt = parse_ts(s["timestamp"])
        if not s_dt:
            continue
        diff = abs((photo_dt - s_dt).total_seconds())
        if diff < best_diff:
            best_diff = diff
            best = s

    if best and best_diff <= max_diff_sec:
        return best
    return None

# ══════════════════════════════════════════════
#  HTTP Handler
# ══════════════════════════════════════════════
class Handler(BaseHTTPRequestHandler):

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        data = self.rfile.read(length)
        ts = self.headers.get("X-Timestamp", "unknown")

        # Windows 파일명에 ':' 사용 불가 → '-'로 치환
        safe_ts = ts.replace(":", "-")
        os.makedirs("images", exist_ok=True)
        filename = f"images/photo_{safe_ts}.jpg"

        with open(filename, "wb") as f:
            f.write(data)

        with log_lock:
            photo_log.append({"timestamp": ts, "file": filename})

        print(f"[HTTP] 사진 저장: {filename} ({len(data)} bytes)")

        self.send_response(200)
        self.end_headers()
        self.wfile.write(b"OK")

    def do_GET(self):
        if self.path == "/gallery":
            self._serve_gallery()
        elif self.path.startswith("/images/"):
            self._serve_image()
        elif self.path == "/api/data":
            self._serve_json()
        else:
            self._serve_index()

    # ── 메인 페이지 ──
    def _serve_index(self):
        html = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>ESP32-CAM Server</title></head>
<body style="font-family:sans-serif; padding:20px;">
<h2>ESP32-CAM + VEML7700 Server</h2>
<p><a href="/gallery">Gallery (사진 + 조도)</a></p>
<p><a href="/api/data">JSON API</a></p>
</body></html>"""
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.end_headers()
        self.wfile.write(html.encode())

    # ── 갤러리 페이지 ──
    def _serve_gallery(self):
        with log_lock:
            photos = sorted(photo_log, key=lambda x: x["timestamp"], reverse=True)
            sensor_count = len(sensor_log)

        rows = ""
        for photo in photos:
            matched = find_closest_sensor(photo["timestamp"])
            if matched:
                lux_cell = f'<span style="color:#f5c518;font-size:1.3em;font-weight:bold;">{matched["lux"]:.1f} lux</span>'
            else:
                lux_cell = '<span style="color:#888;">데이터 없음</span>'

            rows += f"""<tr>
                <td><img src="/{photo['file']}" width="320" style="border-radius:6px;"></td>
                <td style="text-align:center;">{lux_cell}</td>
                <td>{photo['timestamp']}</td>
            </tr>"""

        html = f"""<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<title>ESP32-CAM Gallery</title>
<meta http-equiv="refresh" content="15">
<style>
  body {{ font-family: sans-serif; background: #1a1a2e; color: #eee; padding: 20px; }}
  h2 {{ color: #e94560; }}
  a {{ color: #0f94d2; }}
  table {{ border-collapse: collapse; width: 100%; }}
  th {{ background: #16213e; padding: 12px; text-align: left; }}
  td {{ padding: 10px; border-bottom: 1px solid #333; vertical-align: middle; }}
</style>
</head><body>
<h2>ESP32-CAM + VEML7700 Gallery</h2>
<p>사진: {len(photos)}장 | 센서 데이터: {sensor_count}건 | 
<a href="/gallery">새로고침</a> (15초마다 자동)</p>
<table>
<tr><th>사진</th><th>조도</th><th>타임스탬프</th></tr>
{rows}
</table>
</body></html>"""

        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.end_headers()
        self.wfile.write(html.encode())

    # ── 이미지 서빙 ──
    def _serve_image(self):
        fname = self.path.lstrip("/")
        if os.path.exists(fname):
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.end_headers()
            with open(fname, "rb") as f:
                self.wfile.write(f.read())
        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b"Not Found")

    # ── JSON API ──
    def _serve_json(self):
        with log_lock:
            data = {
                "photos": photo_log,
                "sensor": sensor_log,
            }
        payload = json.dumps(data, ensure_ascii=False, indent=2)
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.end_headers()
        self.wfile.write(payload.encode())

    # 로그 출력 간소화
    def log_message(self, format, *args):
        pass  # 기본 HTTP 로그 숨김 (print로 직접 출력하므로)

# ══════════════════════════════════════════════
#  서버 시작
# ══════════════════════════════════════════════
if __name__ == "__main__":
    # MQTT 스레드 시작
    if MQTT_AVAILABLE:
        t = threading.Thread(target=mqtt_thread, daemon=True)
        t.start()

    # HTTP 서버 시작
    server = HTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    print(f"[HTTP] 서버 시작: http://localhost:{HTTP_PORT}")
    print(f"[HTTP] 갤러리:   http://localhost:{HTTP_PORT}/gallery")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n서버 종료")
        server.server_close()