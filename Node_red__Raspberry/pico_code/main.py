"""
Pico MQTT Demo V2 (PCLS)

- Misst Temperatur & Luftfeuchtigkeit (DHT20) und sendet sie per MQTT an Node-RED
- Empfängt Befehle von Node-RED und schaltet eine LED

Benötigte Files auf dem Pico: main.py, mqtt_pcls.py, dht20.py
"""
import time
from machine import I2C, Pin
from mqtt_pcls import MQTT
from dht20 import DHT20

# ---------- Configs: HIER ANPASSEN ----------
# WLAN
SSID     = "iot-ZHAW"
PASSWORD = "5ne25-5568-zFhHZ"

# MQTT Broker (öffentlich, ohne Login)
BROKER = "test.mosquitto.org"
# BROKER = "broker.hivemq.com"   # Alternative, falls mosquitto nicht erreichbar

# MQTT Topics: BASE_TOPIC ändern, z.B. "zhaw/pcls/<gruppenname>"!
# Gleiche Topics im Node-RED Flow eintragen.
BASE_TOPIC    = "zhaw/pcls/demo"
SEND_TOPIC    = BASE_TOPIC + "/from_pico"   # Pico -> Node-RED (Sensordaten)
RECEIVE_TOPIC = BASE_TOPIC + "/to_pico"     # Node-RED -> Pico (Befehle)

INTERVAL_S = 1.0   # Sekunden zwischen zwei Messungen

# ---------- Setup ----------
mqtt = MQTT(SSID, PASSWORD, BROKER, SEND_TOPIC, RECEIVE_TOPIC)

led = Pin(16, Pin.OUT)   # externe LED an GP16

i2c = I2C(1, sda=Pin(6), scl=Pin(7), freq=400000)   # Grove I2C1
sensor = DHT20(i2c)

print("MQTT demo is on!")

# ---------- Loop ----------
while True:
    # 1. Messen und senden
    temp, humid = sensor.measure()
    mqtt.send({"temp": temp, "humid": humid})

    # 2. Neue Nachricht von Node-RED? (None = nichts Neues)
    msg = mqtt.get_last_message()
    if msg is not None:
        print("Empfangen:", msg)
        if isinstance(msg, dict) and "switch" in msg:
            led.value(msg["switch"])

    time.sleep(INTERVAL_S)
