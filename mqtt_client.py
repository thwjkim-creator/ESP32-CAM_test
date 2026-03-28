import paho.mqtt.client as mqtt
import requests
import json

SERVER_URL = "http://localhost:8080/sensor"

def on_message(client, userdata, msg):
    data = json.loads(msg.payload.decode())
    print(f"조도 수신: {data['lux']} lux")
    # HTTP 서버로 센서 데이터 전달
    requests.post(SERVER_URL, json=data)

client = mqtt.Client()
client.on_message = on_message
client.connect("test.mosquitto.org", 1883)
client.subscribe("sensor/light")
client.loop_forever()