# Pico MQTT Demo V2

Der Pico misst Temperatur und Luftfeuchtigkeit und schickt sie per MQTT an Node-RED.
Node-RED zeigt die Werte im Dashboard an und schaltet über einen Schalter eine LED am Pico.

Diese Demo ist die Vorlage für euer Abschlussprojekt.

## So funktioniert's

```
 Pico W                       MQTT Broker                    Node-RED
┌──────────┐  from_pico   ┌──────────────────┐  from_pico  ┌───────────┐
│ DHT20    │ ───────────▶ │ test.mosquitto   │ ──────────▶ │ Dashboard │
│ LED GP16 │ ◀─────────── │ .org             │ ◀────────── │ Schalter  │
└──────────┘   to_pico    └──────────────────┘   to_pico   └───────────┘
```

| Topic | Richtung | Inhalt (JSON) |
| --- | --- | --- |
| `zhaw/pcls/demo/from_pico` | Pico → Node-RED | `{"temp": 23.4, "humid": 51.2}` |
| `zhaw/pcls/demo/to_pico` | Node-RED → Pico | `{"switch": true}` |

## Files

| File | Wofür |
| --- | --- |
| `pico_code/main.py` | Euer Programm. Configs (WLAN, Broker, Topics) stehen oben. **Nur hier arbeitet ihr.** |
| `pico_code/mqtt_pcls.py` | WLAN- und MQTT-Verbindung mit LED-Feedback. Nicht ändern. |
| `pico_code/dht20.py` | Treiber für den Temperatur-/Feuchtigkeitssensor. |
| `node-red_flow_pico_mqtt_demo_v2.json` | Node-RED Flow mit Dashboard 2. |

## Hardware

- Raspberry Pi Pico W
- DHT20-Sensor am Grove-Anschluss **I2C1** (SDA = GP6, SCL = GP7)
- LED am Grove-Anschluss **D16** (GP16)
- Spannungsschalter am Grove Shield auf **3V3**

Der blaue DHT11 funktioniert mit diesem Code nicht. Auf dem Sensor muss DHT20 stehen.

### Sensor prüfen

Pico einstecken und im MicroPico-Terminal hinter `>>>` eintippen:

```python
from machine import I2C, Pin
print(I2C(1, sda=Pin(6), scl=Pin(7)).scan())
```

Die Ausgabe `[56]` heisst: Sensor gefunden. Bei `[]` antwortet niemand. Dann Kabel, Port **I2C1** und Sensor-Typ prüfen.

## Schritt 1: Node-RED

1. Palette installieren: Menü ☰ → *Manage palette* → *Install* → `@flowfuse/node-red-dashboard`.
   Falls ihr das schon in W02 gemacht habt, entfällt dieser Schritt.
2. Flow importieren: Menü ☰ → *Import* → `node-red_flow_pico_mqtt_demo_v2.json` auswählen → *Import*.
   Meldet Node-RED, dass Nodes schon existieren, *Import copy* wählen.
3. In den Nodes `from_pico` und `to_pico` das Topic anpassen (siehe Schritt 2).
4. *Deploy* klicken. Erst dann sind die Änderungen aktiv.
   Ein blauer Punkt auf einem Node heisst: geändert, aber noch nicht deployed.
5. Dashboard öffnen: `http://<node-red-adresse>/dashboard` → Seite *Pico MQTT Demo*.

## Schritt 2: Pico

1. Ordner `pico_code` in VS Code öffnen (Extension *MicroPico*) oder Thonny verwenden.
2. In `main.py` oben die Configs prüfen:
   - `SSID` / `PASSWORD`: euer WLAN
   - `BASE_TOPIC`: **eigenen Namen wählen**, z.B. `zhaw/pcls/gruppe3`.
     Der Broker ist öffentlich: alle mit dem gleichen Topic sehen und schalten dieselben Daten.
     Dasselbe Topic auch in Node-RED eintragen.
3. Alle drei Files einzeln auf den Pico hochladen.
   MicroPico: Rechtsklick auf das File → *Upload file to Pico*.
   Nicht *Upload project* verwenden: Das lädt den ganzen geöffneten Ordner auf den Pico.
4. `main.py` starten. In der Konsole erscheint:
   ```
   Successfully connected to iot-ZHAW!
   Connected to MQTT broker at test.mosquitto.org
   Subscribed to zhaw/pcls/demo/to_pico
   MQTT demo is on!
   ```
5. Im Dashboard laufen jetzt die Werte ein. Der Schalter *LED* schaltet die LED.

## Die Onboard-LED zeigt den Verbindungsstatus

| LED | Bedeutung |
| --- | --- |
| blinkt schnell | Fehler, noch nicht verbunden (z.B. falsches Passwort, WLAN nicht gefunden) |
| blinkt langsam | Verbindung wird aufgebaut |
| leuchtet | Verbindung zum MQTT-Broker läuft. Bleibt sie an, hängt der Pico. |
| aus | Verbunden |

Details zum Fehler stehen in der Konsole: `Retry 3/100: ... status: -2, Error: No access point found.`

## Die MQTT-Klasse in drei Zeilen

```python
mqtt = MQTT(SSID, PASSWORD, BROKER, SEND_TOPIC, RECEIVE_TOPIC)   # verbinden
mqtt.send({"temp": 21.5})        # dict → JSON → SEND_TOPIC
msg = mqtt.get_last_message()    # neueste Nachricht von RECEIVE_TOPIC, sonst None
```

`get_last_message()` gibt ein `dict` zurück, wenn die Nachricht JSON ist, sonst einen `str`.
Bricht die Verbindung ab, verbindet sich die Klasse beim nächsten Aufruf selbst neu.

## Troubleshooting

- **`command 'micropico.run' not found` beim Klick auf Run.**
  Die MicroPico-Extension ist beim Start abgestürzt.
  `Cmd/Ctrl+Shift+P` → *Developer: Reload Window*.
- **`OSError: [Errno 5] EIO` beim Start von `main.py`.**
  Der Sensor antwortet nicht. I2C-Scan aus dem Abschnitt *Sensor prüfen* ausführen.
- **Node-RED zeigt beim MQTT-Node *disconnected* oder der Pico meldet `Failed to create MQTT client`.**
  Die öffentlichen Broker sind nicht immer erreichbar.
  Dann den anderen Broker eintragen, in `main.py` **und** im Broker-Node von Node-RED:
  `test.mosquitto.org` oder `broker.hivemq.com`.
- **Dashboard bleibt leer, obwohl der Pico sendet.**
  Das Topic in `main.py` und in den beiden MQTT-Nodes vergleichen. Danach *Deploy*.
- **Pico findet das iPhone-Hotspot nicht.**
  Auf dem iPhone *Persönlicher Hotspot* → *Kompatibilität maximieren* einschalten.
- **Pico verbindet sich nicht mit dem Hotspot, obwohl Passwort stimmt.**
  Der Hotspot-Name enthält oft ein typografisches Apostroph (`Harry’s iPhone`).
  Hotspot umbenennen, z.B. `harrys-iphone`.
- **Pico verbindet sich nicht mit `iot-ZHAW`.**
  Die MAC-Adresse des Picos muss bei der ZHAW registriert sein.
  Auslesen mit `_repo_shared/pico/sw08/get_mac_address.py`, dann bei uns melden.
- **LED schaltet sich beim Neustart des Picos wieder ein.**
  Das ist Absicht: Node-RED sendet den Schalterzustand mit *retain*.
  Der Broker merkt sich die letzte Nachricht und schickt sie jedem neuen Abonnenten sofort.

## Weiter fürs Projekt

- Eigenen Sensor einbauen: In `main.py` messen und weitere Werte ins `dict` packen, z.B. `{"temp": t, "co2": c}`.
  In Node-RED die neuen Werte mit `payload.co2` auslesen.
- Eigene Befehle: In Node-RED weitere Schlüssel senden, z.B. `{"color": "red"}`, und in `main.py` auswerten.
- `mqtt_pcls.py` und `dht20.py` müsst ihr dafür nicht ändern.
