# 6 PID-Regelung

> Quelle: ADLS Tutorial (ZHAW), https://github.zhaw.ch/pages/ADLS-Projekte-Labs/PCLS/PID-Regelung.html  
> Gespeichert am 30.10.2026

## 6.1 1. PID-Regelung: Theorie und Praxis

### 6.1.1 1.1 Was ist ein Regelkreis?

Ein **Regelkreis** (engl. *closed-loop control system*) ist ein System, das einen Prozess automatisch steuert, indem es kontinuierlich den Ist-Zustand mit dem Soll-Zustand vergleicht und entsprechend korrigiert. Im Gegensatz zur einfachen Steuerung (offener Regelkreis) erhält der Regler hier eine **Rückmeldung** (Feedback) über den aktuellen Zustand.

```
    Sollwert (w)     Regelabweichung (e)      Stellgrösse (u)      Regelgrösse (y)
         │                  │                      │                   │
         ▼                  ▼                      ▼                   ▼
    ┌─────────┐        ┌─────────┐           ┌─────────┐         ┌─────────┐
    │ Sollwert├───(+)──┤ Regler  ├───────────┤ Strecke ├─────────┤ Ausgang │
    │   w     │    │   │  (PID)  │           │ (System)│         │   y     │
    └─────────┘    │   └─────────┘           └─────────┘         └────┬────┘
                   │                                                  │
                  (-)◄────────────────────────────────────────────────┘
                              Rückführung (Messung: Sensor)
```

#### 6.1.1.1 Wichtige Begriffe

| Begriff (Deutsch)         | Begriff (Englisch)   | Symbol  | Beschreibung                     |
| ------------------------- | -------------------- | ------- | -------------------------------- |
| Führungsgrösse / Sollwert | Setpoint / Reference | w(t)    | Gewünschter Zustand des Systems  |
| Regelgrösse / Istwert     | Controlled Variable  | y(t)    | Aktueller, gemessener Zustand    |
| Regelabweichung           | Error                | e(t)    | Differenz: e(t) = w(t) - y(t)    |
| Stellgrösse               | Control Signal       | u(t)    | Ausgabe des Reglers an den Aktor |
| Rückführung               | Feedback             | y\_M(t) | Gemessener Wert vom Sensor       |

### 6.1.2 1.2 Der PID-Regler

Der **PID-Regler** ist der am häufigsten eingesetzte Regler in der Industrie. Er kombiniert drei Komponenten:

  - **P** = Proportional (reagiert auf aktuellen Fehler)
  - **I** = Integral (reagiert auf vergangene Fehler)
  - **D** = Differential (reagiert auf Fehleränderung)

#### 6.1.2.1 Die PID-Regelgleichung

Die Stellgrösse $u(t)$ setzt sich aus drei Anteilen zusammen:

$$
u(t) = \underbrace{K_p \cdot e(t)}_{\text{P-Anteil}} + \underbrace{K_i \cdot \int_0^t e(\tau) d\tau}_{\text{I-Anteil}} + \underbrace{K_d \cdot \frac{de(t)}{dt}}_{\text{D-Anteil}}
$$

Wobei:

  - $K_p$ = Proportionalverstärkung
  - $K_i$ = Integralverstärkung
  - $K_d$ = Differenzialverstärkung
  - $e(t)$ = Regelabweichung zum Zeitpunkt t

### 6.1.3 1.3 Die drei Komponenten im Detail

#### 6.1.3.1 P-Anteil (Proportional)

$$
u_P = K_p \cdot e(t)
$$

Der P-Anteil reagiert **proportional zur aktuellen Regelabweichung**:

  - Grosse Abweichung → Grosse Stellgrösse
  - Kleine Abweichung → Kleine Stellgrösse
  - Keine Abweichung → Keine Stellgrösse

| Vorteile                              | Nachteile                                               |
| ------------------------------------- | ------------------------------------------------------- |
| Schnelle Reaktion auf Störungen       | **Bleibende Regelabweichung** bei Systemen mit Trägheit |
| Einfach zu verstehen und einzustellen | Zu hoher $K_p$ führt zu Schwingungen              |

#### 6.1.3.2 I-Anteil (Integral)

$$
u_I = K_i \cdot \int_0^t e(\tau) d\tau
$$

In diskreter Form (für Mikrocontroller):

$$
u_I[n] = K_i \cdot \sum_{k=0}^{n} e[k] \cdot \Delta t
$$

Der I-Anteil **summiert die Regelabweichung über die Zeit** auf:

| Vorteile                                 | Nachteile                                                              |
| ---------------------------------------- | ---------------------------------------------------------------------- |
| **Eliminiert bleibende Regelabweichung** | Verlangsamt die Regelung                                               |
| Erkennt auch kleine, anhaltende Fehler   | **Wind-up Problem**: Bei Sättigung kann sich der I-Anteil aufschaukeln |

**Anti-Windup**: Um das Wind-up Problem zu vermeiden, wird die Integration gestoppt, wenn die Stellgrösse ihre Grenzen erreicht.

#### 6.1.3.3 D-Anteil (Differential)

$$
u_D = K_d \cdot \frac{de(t)}{dt}
$$

In diskreter Form:

$$
u_D[n] = K_d \cdot \frac{e[n] - e[n-1]}{\Delta t}
$$

Der D-Anteil reagiert auf die **Änderungsgeschwindigkeit** der Regelabweichung:

| Vorteile                                         | Nachteile                                           |
| ------------------------------------------------ | --------------------------------------------------- |
| **Dämpft Schwingungen** (antizipiert Änderungen) | **Verstärkt Messrauschen**                          |
| Beschleunigt die Regelung                        | Kann bei verrauschten Signalen Probleme verursachen |

### 6.1.4 1.4 Signalglättung und Rauschunterdrückung

Bei realen Sensoren enthält das Messsignal **Rauschen**. Der D-Anteil verstärkt dieses Rauschen erheblich, da er die Ableitung berechnet. Kleine Schwankungen im Sensorsignal führen zu grossen Ausschlägen in der Stellgrösse.

#### 6.1.4.1 Lösung 1: Exponential Smoothing (Gleitender Tiefpass)

$$
y^*[n] = \alpha \cdot y_M[n] + (1-\alpha) \cdot y^*[n-1]
$$

  - $\alpha$ nahe 1: Wenig Glättung, schnelle Reaktion
  - $\alpha$ nahe 0: Starke Glättung, aber **Verzögerung (Totzeit)**

#### 6.1.4.2 Lösung 2: Moving Average (Gleitender Mittelwert)

$$
y^*[n] = \frac{1}{w} \sum_{k=0}^{w-1} y_M[n-k]
$$

  - $w$ = Fensterbreite (z.B. 5 Messungen)
  - Grösseres Fenster = mehr Glättung, aber mehr Verzögerung

> **Warning: Balance finden!**

Zu viel Filterung führt zu **Totzeit** (Verzögerung), was die Regelung instabil machen kann. Empfehlung für typische Anwendungen: $\alpha \approx 0.3-0.5$ oder Fenster $w = 3-5$.

### 6.1.5 1.5 Tuning-Methoden

#### 6.1.5.1 Manuelle Einstellung (Faustregeln)

1.  Starten Sie mit $K_i = K_d = 0$
2.  Erhöhen Sie $K_p$ bis das System schwingt
3.  Reduzieren Sie $K_p$ auf \~60% dieses Wertes
4.  Fügen Sie $K_i$ hinzu um bleibende Abweichung zu eliminieren
5.  Fügen Sie $K_d$ hinzu um Schwingungen zu dämpfen

#### 6.1.5.2 Ziegler-Nichols Methode

1.  Setze $K_i = K_d = 0$
2.  Erhöhe $K_p$ bis Dauerschwingung mit kritischer Verstärkung $K_u$ entsteht
3.  Miss die Schwingungsdauer $T_u$
4.  Berechne Parameter:

| Reglertyp | $K_p$ | $K_i$ | $K_d$ |
| --------- | ------------ | ------------ | ------------ |
| P         | $0.5 \cdot K_u$ | \-           | \-           |
| PI        | $0.45 \cdot K_u$ | $1.2 \cdot K_p / T_u$ | \-           |
| PID       | $0.6 \cdot K_u$ | $2 \cdot K_p / T_u$ | $K_p \cdot T_u / 8$ |

### 6.1.6 1.6 Typische Probleme und Lösungen

| Problem                 | Mögliche Ursache                              | Lösung                                            |
| ----------------------- | --------------------------------------------- | ------------------------------------------------- |
| Oszillation             | $K_p$ zu hoch                          | $K_p$ reduzieren, $K_d$ erhöhen     |
| Überschwingen           | $K_p$ zu hoch, $K_d$ zu niedrig | $K_p$ senken, $K_d$ erhöhen         |
| Langsame Reaktion       | $K_p$ zu niedrig                       | $K_p$ erhöhen                              |
| Bleibende Abweichung    | $K_i$ zu niedrig oder 0                | $K_i$ erhöhen                              |
| Rauschen im Stellsignal | $K_d$ zu hoch                          | $K_d$ reduzieren, Tiefpassfilter einsetzen |

### 6.1.7 1.7 Zusammenfassung

| Anteil | Formel       | Wirkung                          | Zu hoch?             | Zu niedrig?        |
| ------ | ------------ | -------------------------------- | -------------------- | ------------------ |
| **P**  | $K_p \cdot e$ | Reagiert auf aktuelle Abweichung | Oszillation          | Langsam            |
| **I**  | $K_i \cdot \int e \, dt$ | Eliminiert bleibende Abweichung  | Wind-up, Oszillation | Stationärer Fehler |
| **D**  | $K_d \cdot \frac{de}{dt}$ | Dämpft Schwingungen              | Rauschen             | Überschwingen      |

## 6.2 2. Ball-on-Beam Experiment

### 6.2.1 2.1 Problemstellung

Das **Ball-on-Beam** Experiment ist ein klassisches Beispiel für ein instabiles System, das durch Regelung stabilisiert werden muss. Ein Ball soll auf einer kippbaren Schiene an einer bestimmten Position gehalten werden.

```
          Servo
            │
            ▼
    ┌───────────────────┐
    │   ◯ ←Ball        │  ← TOF Sensor
    └───────────────────┘
            ▲
        Schiene
```

**Hardware-Komponenten:**

  - **Raspberry Pi Pico**: Mikrocontroller für die Regelung
  - **Grove TOF Sensor (VL53L0X)**: Misst die Position des Balls (Time-of-Flight)
  - **Servo-Motor**: Kippt die Schiene

### 6.2.2 2.2 Regelkreis beim Ball-on-Beam

| Komponente          | Im Experiment                        |
| ------------------- | ------------------------------------ |
| Sollwert (w)        | Zielposition des Balls (z.B. 200 mm) |
| Istwert (y)         | Gemessene Position vom TOF-Sensor    |
| Regelabweichung (e) | Abstand zur Zielposition             |
| Stellgrösse (u)     | Neigungswinkel der Schiene           |
| Aktor               | Servo-Motor                          |
| Sensor              | TOF-Distanzsensor                    |

### 6.2.3 2.3 MicroPython Implementierung

#### 6.2.3.1 Signal-Filter Klasse

```python
class ExponentialFilter:
    """
    Exponential Smoothing Filter zur Rauschunterdrückung.

    Formel: y*[n] = alpha * y_M[n] + (1-alpha) * y*[n-1]
    """
    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.value = None

    def update(self, measurement):
        if self.value is None:
            self.value = measurement
        else:
            self.value = self.alpha * measurement + (1 - self.alpha) * self.value
        return self.value

    def reset(self):
        self.value = None
```

#### 6.2.3.2 PID-Regler Klasse

```python
class PIDController:
    def __init__(self, Kp, Ki, Kd, output_min=-30, output_max=30):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_min = output_min
        self.output_max = output_max

        self.error_sum = 0
        self.error_prev = 0
        self.last_time = time.ticks_ms()

    def reset(self):
        self.error_sum = 0
        self.error_prev = 0
        self.last_time = time.ticks_ms()

    def update(self, setpoint, measured_value):
        # Zeitschritt berechnen
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, self.last_time) / 1000.0
        self.last_time = current_time

        if dt <= 0:
            return 0

        # Regelabweichung
        error = setpoint - measured_value

        # P-Anteil
        P = self.Kp * error

        # I-Anteil
        self.error_sum += error * dt
        I = self.Ki * self.error_sum

        # D-Anteil
        D = self.Kd * (error - self.error_prev) / dt

        output = P + I + D

        # Anti-Windup: Begrenzung
        if output > self.output_max:
            output = self.output_max
            self.error_sum -= error * dt  # Rückgängig machen
        elif output < self.output_min:
            output = self.output_min
            self.error_sum -= error * dt

        self.error_prev = error
        return output
```

#### 6.2.3.3 Hauptprogramm

```python
from machine import Pin, PWM, I2C
import time

# Hardware-Konfiguration
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)
servo = PWM(Pin(15))
servo.freq(50)

# Servo-Kalibrierung
SERVO_MIN_DUTY = 1000
SERVO_MAX_DUTY = 9000

def set_servo_angle(angle_deg):
    """Setzt den Servo auf einen Winkel (-30 bis +30 Grad)."""
    angle_deg = max(-30, min(30, angle_deg))
    normalized = (angle_deg + 30) / 60.0
    duty = SERVO_MIN_DUTY + normalized * (SERVO_MAX_DUTY - SERVO_MIN_DUTY)
    servo.duty_u16(int(duty))

def main():
    # Filter und PID initialisieren
    distance_filter = ExponentialFilter(alpha=0.3)
    pid = PIDController(Kp=0.15, Ki=0.005, Kd=0.20)

    SETPOINT = 200  # Zielposition in mm

    while True:
        # Position messen und filtern
        raw_position = read_distance()  # TOF-Sensor auslesen
        filtered_position = distance_filter.update(raw_position)

        # PID berechnen
        angle = pid.update(SETPOINT, filtered_position)

        # Servo ansteuern
        set_servo_angle(angle)

        # Regelfrequenz ~50 Hz
        time.sleep_ms(20)
```

### 6.2.4 2.4 Empfohlene Parameter

Für das Ball-on-Beam System haben sich folgende Startwerte bewährt:

| Parameter    | Wert         | Beschreibung            |
| ------------ | ------------ | ----------------------- |
| $K_p$ | 0.10 - 0.20  | Proportionalverstärkung |
| $K_i$ | 0.005 - 0.01 | Integralverstärkung     |
| $K_d$ | 0.15 - 0.25  | Differenzialverstärkung |
| $\alpha$ | 0.3 - 0.5    | Filter-Koeffizient      |

> **Tip: Schrittweises Vorgehen**

1.  Beginnen Sie mit $K_i = K_d = 0$
2.  Erhöhen Sie $K_p$ bis der Ball reagiert, aber nicht zu stark schwingt
3.  Fügen Sie $K_d$ hinzu um Schwingungen zu dämpfen
4.  Fügen Sie bei Bedarf $K_i$ hinzu um bleibende Abweichung zu eliminieren

## 6.3 3. Anwendungen in der Biomedizin

PID-Regelung findet breite Anwendung in biologischen und medizinischen Systemen:

### 6.3.1 Biologische Systeme

  - **Genregulation**: Feedback-Schleifen in der Genexpression
  - **Homöostase**: Temperatur-, pH- und Blutzuckerregulation
  - **Immunantwort**: Regulierung von Entzündungsreaktionen

### 6.3.2 Medizinische Geräte

  - **Herzschrittmacher**: Anpassung der Herzfrequenz an den Bedarf
  - **Insulinpumpen**: Automatische Blutzuckerregulation bei Diabetes
  - **Beatmungsgeräte**: Steuerung von Atemvolumen und -frequenz
  - **Chirurgische Roboter**: Präzise Kraftregelung bei Operationen

> *“Feedback is a central feature of life. The process of feedback governs how we grow, respond to stress and challenge, and regulate factors such as body temperature, blood pressure, and cholesterol level. The concept also applies to ecology, and in time we will come to see that resistance to antibiotics is also a feedback process.”* — Hoagland & Dodson, 2000
