import network, rp2, machine, time, json, socket, struct

"""
MQTT client class for the Pico (by Norman & Samuel) -- Version 2

This module provides a MQTT client class for the Raspberry Pi Pico W that handles:
- WiFi connection with status LED feedback
- MQTT broker connection and message handling
- Basic publish/subscribe functionality
- JSON message encoding/decoding

The LED gives Feedback during the connection
- Fast blink: Error, not (yet) connected
- Slow blink: Connection in progress
- LED off: Connected
- LED on: Got stuck

Usage (see main.py):
    mqtt = MQTT(SSID, PASSWORD, BROKER, SEND_TOPIC, RECEIVE_TOPIC)
    mqtt.send({"temp": 21.5})        # dict/list -> JSON, published on SEND_TOPIC
    msg = mqtt.get_last_message()    # dict (JSON), str or None (nothing new)

This file contains everything needed: no further MQTT library required.
"""

# Define status messages
WLAN_STATUS_MESSAGES = {
    network.STAT_IDLE: "Status: Idle (no connection activity).",  # 0
    network.STAT_CONNECTING: "Status: Connection in progress.",  # 1
    2: "Status: Connect to AP, waiting for IP address.", # 2 (not 100% clear whether text is correct)
    network.STAT_WRONG_PASSWORD: "Error: Incorrect password.", # -1
    network.STAT_NO_AP_FOUND: "Error: No access point found.", # -2
    network.STAT_CONNECT_FAIL: "Error: Connection failed due to other problems.", # -3
    network.STAT_GOT_IP: "Status: Connected successfully with an IP address." # 3
}

class MQTT:

    def __init__(self, wlan_ssid, wlan_password, mqtt_address, send_topic, receive_topic,
                 wifi_country='CH', **mqtt_kwargs):
        """
        Initialize the MQTT client, connect to WiFi and broker, subscribe to receive_topic

        Args:
            wlan_ssid (str): SSID of the Wi-Fi network
            wlan_password (str): Password of the Wi-Fi network
            mqtt_address (str): Address of the MQTT broker
            send_topic (str): Topic for outgoing messages (Pico -> Node-RED)
            receive_topic (str): Topic for incoming messages (Node-RED -> Pico)
            wifi_country (str): Country code for the Wi-Fi network (default: 'CH')
            **mqtt_kwargs: Additional keyword arguments for the MQTT client
        """
        self.wlan_ssid = wlan_ssid
        self.wlan_password = wlan_password
        self.mqtt_address = mqtt_address
        self.send_topic = send_topic
        self.receive_topic = receive_topic

        # Set the Wi-Fi country
        rp2.country(wifi_country)

        # initiate wlan client
        self.wlan = network.WLAN(network.STA_IF)

        # initiate mqtt client
        self.mqtt_client = MQTTClient(self.client_id(), mqtt_address, **mqtt_kwargs)
        self.mqtt_client.set_callback(self._mqtt_callback)
        self._last_received_msg = None

        # initiate LED on pico
        self.LED = machine.Pin("LED", machine.Pin.OUT)
        self.LED.off()

        # connect
        self.connect()

    # ---- Public API ----

    def send(self, msg):
        """
        Send a message to send_topic

        Args:
            msg (dict, list, tuple, str): Message to send. dict/list/tuple are sent as JSON.

        Returns:
            bool: True if successful, False otherwise
        """
        if type(msg) in [dict, list, tuple]:
            msg = json.dumps(msg)
        try:
            self.mqtt_client.publish(self.send_topic, msg)
            return True
        except Exception as e:
            print(f"MQTT Error: Unable to send message - {e}")
            if not self.connect():
                return False
            try:
                self.mqtt_client.publish(self.send_topic, msg)
                return True
            except Exception as e:
                print(f"MQTT Error: Unable to send message after reconnect - {e}")
                return False

    def get_last_message(self):
        """
        Get the newest message received on receive_topic since the last call

        Returns:
            dict/list (if JSON), str (otherwise) or None (no new message)
        """
        try:
            self._check_all_msgs()
        except Exception as e:
            print(f"MQTT Error: Unable to check messages - {e}")
            if self.connect():
                try:
                    self._check_all_msgs()
                except Exception as e:
                    print(f"MQTT Error: Unable to check messages after reconnect - {e}")

        last_msg = self._last_received_msg
        self._last_received_msg = None
        return last_msg

    def connect(self):
        """
        Connect to the Wi-Fi network and the MQTT broker, subscribe to receive_topic

        Returns:
            bool: True if successful, False otherwise
        """
        if not self.ensure_wlan():
            return False
        if not self.connect_mqtt():
            return False
        try:
            self.mqtt_client.subscribe(self.receive_topic)
            print(f"Subscribed to {self.receive_topic}")
            return True
        except Exception as e:
            print(f"MQTT Error: Unable to subscribe to topic {self.receive_topic} - {e}")
            return False

    # ---- Helper functions ----

    def _check_all_msgs(self, max_msgs=20):
        """
        Process all pending messages; only the newest one is kept
        """
        for _ in range(max_msgs):
            if self.mqtt_client.check_msg() is None:
                break

    def _mqtt_callback(self, topic, msg):
        """
        Callback function for incoming messages

        Args:
            topic: Topic of the incoming message
            msg : Message of the incoming message
        """
        msg = msg.decode()
        try:
            self._last_received_msg = json.loads(msg)
        except Exception:
            self._last_received_msg = msg  # not JSON: keep as string

    def client_id(self):
        """
        Create a unique identifier for the MQTT client
        """
        pattern = "{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}"
        return pattern.format(*machine.unique_id())

    def blink_led(self, frequency_hz=5, duration_seconds=1):
        """
        Blink the LED at the specified frequency for the given duration.

        Args:
            frequency_hz (float): Frequency of blinking in Hz (default: 5)
            duration_seconds (float): Total duration to blink in seconds (default: 1)
        """
        cycles = int(duration_seconds * frequency_hz)
        half_period = 0.5 / frequency_hz

        for _ in range(cycles):
            self.LED.on()
            time.sleep(half_period)
            self.LED.off()
            time.sleep(half_period)
        self.LED.off()

    def ensure_wlan(self, max_retries=100, sleep_seconds=2):
        """
        Ensure the Wi-Fi connection is established

        Args:
            max_retries (int): Maximum number of retries (default: 100)
            sleep_seconds (float): Time to sleep between retries in seconds (default: 2)

        Returns:
            bool: True if successful, False otherwise
        """
        # WLAN should already be initialized in __init__
        self.wlan.active(True)

        # Already connected: nothing to do
        if self.wlan.isconnected():
            return True

        # Set Wi-Fi power-saving mode off
        self.wlan.config(pm=0xa11140)

        # Start the connection process
        self.wlan.connect(self.wlan_ssid, self.wlan_password)
        retries = 0

        # Retry loop
        while not self.wlan.isconnected() and retries < max_retries:
            status = self.wlan.status()
            status_msg = WLAN_STATUS_MESSAGES.get(status, f"Unknown status code {status}")
            print(f"Retry {retries}/{max_retries}: {self.wlan_ssid} connected: {self.wlan.isconnected()}, status: {status}, {status_msg}")

            retries += 1

            if status < 0:
                self.wlan.connect(self.wlan_ssid, self.wlan_password)
                self.blink_led(frequency_hz=10, duration_seconds=sleep_seconds)  # fast blinking
            else:
                self.blink_led(frequency_hz=2, duration_seconds=sleep_seconds)  # slow blinking

        # Final check
        if self.wlan.isconnected():
            print(f"Successfully connected to {self.wlan_ssid}!")
            print(f"IP Configuration: {self.wlan.ifconfig()}")
            try:
                # Retrieve and print RSSI if supported
                rssi = self.wlan.status('rssi')
                print(f"Signal Strength (RSSI): {rssi} dBm")
            except Exception as e:
                print(f"Unable to retrieve RSSI: {e}")
            return True
        else:
            print(f"Failed to connect to {self.wlan_ssid} after multiple attempts.")
            return False

    def connect_mqtt(self):
        """
        Initialize MQTT client and connect to broker

        Returns:
            bool: True if successful, False otherwise
        """
        self.LED.on()
        try:
            self.mqtt_client.connect()
            print(f"Connected to MQTT broker at {self.mqtt_address}")
            self.LED.off()
            return True
        except Exception as e:
            print(f"Failed to create MQTT client: {e}")
            self.LED.off()
            return False


# =====================================================================
# Library-Code (umqtt.simple, MicroPython-Standard).
# Muss nicht gelesen werden.
# =====================================================================
class MQTTException(Exception):
    pass


class MQTTClient:
    def __init__(
        self,
        client_id,
        server,
        port=0,
        user=None,
        password=None,
        keepalive=0,
        ssl=None,
    ):
        if port == 0:
            port = 8883 if ssl else 1883
        self.client_id = client_id
        self.sock = None
        self.server = server
        self.port = port
        self.ssl = ssl
        self.pid = 0
        self.cb = None
        self.user = user
        self.pswd = password
        self.keepalive = keepalive
        self.lw_topic = None
        self.lw_msg = None
        self.lw_qos = 0
        self.lw_retain = False

    def _send_str(self, s):
        self.sock.write(struct.pack("!H", len(s)))
        self.sock.write(s)

    def _recv_len(self):
        n = 0
        sh = 0
        while 1:
            b = self.sock.read(1)[0]
            n |= (b & 0x7F) << sh
            if not b & 0x80:
                return n
            sh += 7

    def set_callback(self, f):
        self.cb = f

    def set_last_will(self, topic, msg, retain=False, qos=0):
        assert 0 <= qos <= 2
        assert topic
        self.lw_topic = topic
        self.lw_msg = msg
        self.lw_qos = qos
        self.lw_retain = retain

    def connect(self, clean_session=True, timeout=None):
        self.sock = socket.socket()
        self.sock.settimeout(timeout)
        addr = socket.getaddrinfo(self.server, self.port)[0][-1]
        self.sock.connect(addr)
        if self.ssl:
            self.sock = self.ssl.wrap_socket(self.sock, server_hostname=self.server)
        premsg = bytearray(b"\x10\0\0\0\0\0")
        msg = bytearray(b"\x04MQTT\x04\x02\0\0")

        sz = 10 + 2 + len(self.client_id)
        msg[6] = clean_session << 1
        if self.user:
            sz += 2 + len(self.user) + 2 + len(self.pswd)
            msg[6] |= 0xC0
        if self.keepalive:
            assert self.keepalive < 65536
            msg[7] |= self.keepalive >> 8
            msg[8] |= self.keepalive & 0x00FF
        if self.lw_topic:
            sz += 2 + len(self.lw_topic) + 2 + len(self.lw_msg)
            msg[6] |= 0x4 | (self.lw_qos & 0x1) << 3 | (self.lw_qos & 0x2) << 3
            msg[6] |= self.lw_retain << 5

        i = 1
        while sz > 0x7F:
            premsg[i] = (sz & 0x7F) | 0x80
            sz >>= 7
            i += 1
        premsg[i] = sz

        self.sock.write(premsg, i + 2)
        self.sock.write(msg)
        # print(hex(len(msg)), hexlify(msg, ":"))
        self._send_str(self.client_id)
        if self.lw_topic:
            self._send_str(self.lw_topic)
            self._send_str(self.lw_msg)
        if self.user:
            self._send_str(self.user)
            self._send_str(self.pswd)
        resp = self.sock.read(4)
        assert resp[0] == 0x20 and resp[1] == 0x02
        if resp[3] != 0:
            raise MQTTException(resp[3])
        return resp[2] & 1

    def disconnect(self):
        self.sock.write(b"\xe0\0")
        self.sock.close()

    def ping(self):
        self.sock.write(b"\xc0\0")

    def publish(self, topic, msg, retain=False, qos=0):
        pkt = bytearray(b"\x30\0\0\0")
        pkt[0] |= qos << 1 | retain
        sz = 2 + len(topic) + len(msg)
        if qos > 0:
            sz += 2
        assert sz < 2097152
        i = 1
        while sz > 0x7F:
            pkt[i] = (sz & 0x7F) | 0x80
            sz >>= 7
            i += 1
        pkt[i] = sz
        # print(hex(len(pkt)), hexlify(pkt, ":"))
        self.sock.write(pkt, i + 1)
        self._send_str(topic)
        if qos > 0:
            self.pid += 1
            pid = self.pid
            struct.pack_into("!H", pkt, 0, pid)
            self.sock.write(pkt, 2)
        self.sock.write(msg)
        if qos == 1:
            while 1:
                op = self.wait_msg()
                if op == 0x40:
                    sz = self.sock.read(1)
                    assert sz == b"\x02"
                    rcv_pid = self.sock.read(2)
                    rcv_pid = rcv_pid[0] << 8 | rcv_pid[1]
                    if pid == rcv_pid:
                        return
        elif qos == 2:
            assert 0

    def subscribe(self, topic, qos=0):
        assert self.cb is not None, "Subscribe callback is not set"
        pkt = bytearray(b"\x82\0\0\0")
        self.pid += 1
        struct.pack_into("!BH", pkt, 1, 2 + 2 + len(topic) + 1, self.pid)
        # print(hex(len(pkt)), hexlify(pkt, ":"))
        self.sock.write(pkt)
        self._send_str(topic)
        self.sock.write(qos.to_bytes(1, "little"))
        while 1:
            op = self.wait_msg()
            if op == 0x90:
                resp = self.sock.read(4)
                # print(resp)
                assert resp[1] == pkt[2] and resp[2] == pkt[3]
                if resp[3] == 0x80:
                    raise MQTTException(resp[3])
                return

    # Wait for a single incoming MQTT message and process it.
    # Subscribed messages are delivered to a callback previously
    # set by .set_callback() method. Other (internal) MQTT
    # messages processed internally.
    def wait_msg(self):
        res = self.sock.read(1)
        self.sock.setblocking(True)
        if res is None:
            return None
        if res == b"":
            raise OSError(-1)
        if res == b"\xd0":  # PINGRESP
            sz = self.sock.read(1)[0]
            assert sz == 0
            return None
        op = res[0]
        if op & 0xF0 != 0x30:
            return op
        sz = self._recv_len()
        topic_len = self.sock.read(2)
        topic_len = (topic_len[0] << 8) | topic_len[1]
        topic = self.sock.read(topic_len)
        sz -= topic_len + 2
        if op & 6:
            pid = self.sock.read(2)
            pid = pid[0] << 8 | pid[1]
            sz -= 2
        msg = self.sock.read(sz)
        self.cb(topic, msg)
        if op & 6 == 2:
            pkt = bytearray(b"\x40\x02\0\0")
            struct.pack_into("!H", pkt, 2, pid)
            self.sock.write(pkt)
        elif op & 6 == 4:
            assert 0
        return op

    # Checks whether a pending message from server is available.
    # If not, returns immediately with None. Otherwise, does
    # the same processing as wait_msg.
    def check_msg(self):
        self.sock.setblocking(False)
        return self.wait_msg()
