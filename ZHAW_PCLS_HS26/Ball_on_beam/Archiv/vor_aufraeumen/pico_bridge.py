"""
pico_bridge.py: Brücke PC <-> Pico für Claude (Ball on Beam)

Läuft auf Jans Windows-PC. Wartet auf Aufträge im Ordner jobs/, lädt Dateien
auf den Pico, führt Code aus und schreibt die Ausgabe live nach results/.

Start:   python pico_bridge.py            (Port automatisch)
         python pico_bridge.py COM5       (Port fest)
Stopp:   Ctrl+C im Terminal

Vorher in VS Code MicroPico trennen (Disconnect), sonst ist der Port belegt.
Braucht pyserial:  pip install pyserial

Auftrag jobs/<name>.job (JSON):
  {"upload": ["Scripts/bob/bob.py", ...],   Pfade relativ zu Ball_on_beam, landen im Pico-Root
   "run": "import bob; bob.hello()",         Python-Code für den Pico
   "timeout": 120}                           Sekunden
Ergebnis: results/<name>.log (live), results/<name>.done (fertig, mit Status)
Notstopp: Datei STOP im Bridge-Ordner anlegen -> Ctrl-C an den Pico
"""
import base64
import json
import os
import sys
import time

import serial
import serial.tools.list_ports

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                     # Ball_on_beam
JOBS = os.path.join(HERE, "jobs")
DONE = os.path.join(JOBS, "done")
RESULTS = os.path.join(HERE, "results")
STATUS = os.path.join(HERE, "status.txt")
STOPFILE = os.path.join(HERE, "STOP")


def find_port():
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x2E8A:                      # Raspberry Pi
            return p.device
    return None


def status(msg):
    try:
        with open(STATUS, "w", encoding="utf-8") as f:
            f.write("%s %s\n" % (time.strftime("%Y-%m-%d %H:%M:%S"), msg))
    except OSError:
        pass


class Pico:
    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=0.05)

    def read_until(self, end, timeout, cb=None):
        buf = b""
        t0 = time.time()
        while not buf.endswith(end):
            c = self.ser.read(1)
            if c:
                buf += c
                if cb:
                    cb(c)
            elif time.time() - t0 > timeout:
                raise TimeoutError(buf[-200:])
        return buf

    def enter_raw(self):
        self.ser.write(b"\r\x03")
        time.sleep(0.1)
        self.ser.write(b"\x03")
        time.sleep(0.2)
        self.ser.reset_input_buffer()
        self.ser.write(b"\r\x01")
        self.read_until(b"raw REPL; CTRL-B to exit\r\n>", 5)

    def soft_reset(self):
        # im raw REPL: Neustart ohne main.py
        self.ser.write(b"\x04")
        self.read_until(b"soft reboot\r\n", 5)
        self.read_until(b"raw REPL; CTRL-B to exit\r\n>", 5)

    def exec(self, code, timeout, out=None):
        """Code ausführen, stdout live an out(bytes). Gibt (ok, stderr) zurück."""
        data = code.encode("utf-8")
        for i in range(0, len(data), 256):
            self.ser.write(data[i:i + 256])
            time.sleep(0.01)
        self.ser.write(b"\x04")
        self.read_until(b"OK", 5)
        t0 = time.time()
        stdout = b""
        while True:
            if os.path.exists(STOPFILE):
                os.remove(STOPFILE)
                self.ser.write(b"\x03")
                if out:
                    out(b"\n[BRIDGE] STOP-Datei: Ctrl-C gesendet\n")
            if time.time() - t0 > timeout:
                self.ser.write(b"\x03")
                if out:
                    out(b"\n[BRIDGE] Timeout: Ctrl-C gesendet\n")
                t0 = time.time() + 1e9           # nur einmal
            c = self.ser.read(256)
            if not c:
                continue
            if b"\x04" in c:
                k = c.index(b"\x04")
                if out:
                    out(c[:k])
                rest = c[k + 1:]
                err = rest
                while err.count(b"\x04") < 1:
                    err += self.read_until(b"\x04", 10)
                k2 = err.index(b"\x04")
                tail = err[k2 + 1:]
                err = err[:k2]
                if b">" not in tail:
                    self.read_until(b">", 5)
                return (len(err) == 0, err.decode("utf-8", "replace"))
            if out:
                out(c)

    def upload(self, local, remote):
        with open(local, "rb") as f:
            data = f.read()
        ok, err = self.exec("import ubinascii as _b\n_f=open(%r,'wb')" % remote, 10)
        if not ok:
            raise RuntimeError(err)
        for i in range(0, len(data), 768):
            chunk = base64.b64encode(data[i:i + 768]).decode()
            ok, err = self.exec("_f.write(_b.a2b_base64(%r))" % chunk, 10)
            if not ok:
                raise RuntimeError(err)
        ok, err = self.exec("_f.close()", 10)
        if not ok:
            raise RuntimeError(err)
        return len(data)


def run_job(pico, path):
    name = os.path.splitext(os.path.basename(path))[0]
    log_path = os.path.join(RESULTS, name + ".log")
    with open(path, encoding="utf-8") as f:
        job = json.load(f)
    status("RUN " + name)
    t0 = time.time()
    with open(log_path, "wb") as log:
        def out(b):
            log.write(b)
            log.flush()
        result = "ok"
        try:
            pico.enter_raw()
            pico.soft_reset()
            for rel in job.get("upload", []):
                n = pico.upload(os.path.join(ROOT, rel), os.path.basename(rel))
                out(("[BRIDGE] hochgeladen %s (%d B)\n" % (os.path.basename(rel), n)).encode())
            ok, err = pico.exec(job.get("run", ""), float(job.get("timeout", 120)), out)
            if not ok:
                out(b"\n[BRIDGE] FEHLER:\n" + err.encode())
                result = "error"
        except Exception as ex:                  # Verbindung neu aufbauen lassen
            out(("\n[BRIDGE] AUSNAHME: %r\n" % (ex,)).encode())
            result = "exception"
    with open(os.path.join(RESULTS, name + ".done"), "w", encoding="utf-8") as f:
        f.write("%s %.1f s\n" % (result, time.time() - t0))
    os.replace(path, os.path.join(DONE, os.path.basename(path)))
    print(time.strftime("%H:%M:%S"), name, result, "%.1f s" % (time.time() - t0))
    return result


def main():
    for d in (JOBS, DONE, RESULTS):
        os.makedirs(d, exist_ok=True)
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    if not port:
        print("Kein Pico gefunden. Ist er eingesteckt und MicroPico getrennt?")
        status("KEIN PICO")
        return
    print("Pico an", port, "- warte auf Aufträge in", JOBS, "(Ctrl+C beendet)")
    pico = Pico(port)
    status("BEREIT " + port)
    last = 0
    while True:
        jobs = sorted(f for f in os.listdir(JOBS) if f.endswith(".job"))
        if jobs:
            r = run_job(pico, os.path.join(JOBS, jobs[0]))
            if r == "exception":
                try:
                    pico.ser.close()
                except Exception:
                    pass
                time.sleep(1)
                port = (sys.argv[1] if len(sys.argv) > 1 else find_port()) or port
                try:
                    pico = Pico(port)
                except Exception as ex:
                    print("Port weg:", ex)
                    status("PORT WEG " + str(ex))
                    time.sleep(3)
                    continue
            status("BEREIT " + port)
        else:
            time.sleep(0.3)
            if time.time() - last > 5:
                status("BEREIT " + port)
                last = time.time()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        status("BEENDET")
        print("beendet")
